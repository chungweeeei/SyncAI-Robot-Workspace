"""Shared HTTP client for the SyncAI backend REST API.

The task and map tool modules are thin clients for ``syncai_backend``'s REST
API; this module holds the one HTTP helper they share so the base URL, timeout,
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


def request(method: str, path: str, json=None, params: dict = None) -> dict:
    """Call the backend and normalize the outcome into a plain dict.

    Returns the parsed JSON body on success, or ``{"error": ...}`` describing a
    transport failure or a non-2xx response (the backend's error body is passed
    through under 'detail' when present).
    """
    url = f"{base_url()}{path}"
    try:
        resp = requests.request(
            method, url, json=json, params=params, timeout=HTTP_TIMEOUT
        )
    except requests.RequestException as exc:
        return {"error": f"Failed to reach backend at {url}: {exc}"}

    try:
        body = resp.json()
    except ValueError:
        body = {"detail": resp.text}

    if not resp.ok:
        return {
            "error": f"Backend returned HTTP {resp.status_code} for {method} {path}.",
            "status_code": resp.status_code,
            "detail": body.get("detail", body) if isinstance(body, dict) else body,
        }

    return body
