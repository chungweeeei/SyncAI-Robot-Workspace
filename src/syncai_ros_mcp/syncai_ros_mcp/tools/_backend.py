"""Shared HTTP client for the SyncAI backend REST API.

The task and map tool modules are thin clients for ``syncai_backend``'s REST
API; this module holds the HTTP helpers they share so the base URL, timeout,
and error-normalization behaviour live in a single place.

The backend base URL defaults to ``http://localhost:3000`` (the port
``syncai_backend`` binds in ``interfaces/rest/server.py``) and can be overridden
with the ``SYNCAI_BACKEND_BASE_URL`` environment variable.
"""

import os

import requests


# Bound on every backend call so a hung request can't wedge the MCP tool.
HTTP_TIMEOUT = 10.0


def base_url() -> str:
    """Backend base URL; loopback by default, overridable via the environment."""
    return os.environ.get("SYNCAI_BACKEND_BASE_URL", "http://localhost:3000")


def _send(method: str, path: str, json=None, params: dict = None):
    """Issue the request; return the Response, or an ``{"error": ...}`` dict.

    Only the transport failure is normalized here. Turning a non-2xx status
    into an error dict needs the body, and JSON and binary endpoints read it
    differently, so that half lives in the callers.
    """
    url = f"{base_url()}{path}"
    try:
        return requests.request(
            method, url, json=json, params=params, timeout=HTTP_TIMEOUT
        )
    except requests.RequestException as exc:
        return {"error": f"Failed to reach backend at {url}: {exc}"}


def _error(resp: requests.Response, method: str, path: str) -> dict:
    """Normalize a non-2xx response into the shared error dict.

    The backend's error body is passed through under 'detail' when present
    (FastAPI puts its message there); a non-JSON body is passed through as text
    so a proxy's HTML 502 page still says something.
    """
    try:
        body = resp.json()
    except ValueError:
        body = {"detail": resp.text}

    return {
        "error": f"Backend returned HTTP {resp.status_code} for {method} {path}.",
        "status_code": resp.status_code,
        "detail": body.get("detail", body) if isinstance(body, dict) else body,
    }


def request(method: str, path: str, json=None, params: dict = None) -> dict:
    """Call the backend and normalize the outcome into a plain dict.

    Returns the parsed JSON body on success, or ``{"error": ...}`` describing a
    transport failure or a non-2xx response (the backend's error body is passed
    through under 'detail' when present).
    """
    resp = _send(method, path, json=json, params=params)
    if isinstance(resp, dict):
        return resp

    if not resp.ok:
        return _error(resp, method, path)

    try:
        return resp.json()
    except ValueError:
        return {"detail": resp.text}


def request_bytes(method: str, path: str, params: dict = None):
    """Call the backend for a binary body (e.g. a PNG).

    Returns the raw ``bytes`` on a 2xx, or the same ``{"error", "status_code",
    "detail"}`` dict as :func:`request` otherwise. The split exists because
    ``request`` parses JSON: the map image endpoint answers with
    ``image/png`` bytes and has no JSON envelope to parse.
    """
    resp = _send(method, path, params=params)
    if isinstance(resp, dict):
        return resp

    if not resp.ok:
        return _error(resp, method, path)

    return resp.content
