import os
import configparser
from typing import Dict, Optional

import requests
import structlog


# Absolute path so the INI resolves no matter what cwd the process is started
# from — the old relative path only worked because every entrypoint happened to
# run from the workspace root. ~/robot_ws is the workspace inside the robot
# container, where docker-compose bind-mounts the per-robot instance INI over
# config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")

REQUEST_TIMEOUT_S = 5.0


class UnknownArtifactError(Exception):
    """artifact_id has no entry in the [artifacts] registry."""


class ArtifactCommandRejected(Exception):
    """The artifact backend rejected the request (HTTP 4xx)."""


class ArtifactUnavailable(Exception):
    """The artifact backend could not be reached or failed (HTTP 5xx)."""


class ArtifactGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, registry: Dict[str, str]):
        self._logger = logger
        self._registry = {
            artifact_id: base_url.rstrip("/")
            for artifact_id, base_url in registry.items()
        }

    def _base_url(self, artifact_id: str) -> str:
        base_url = self._registry.get(artifact_id)
        if base_url is None:
            raise UnknownArtifactError(
                f"Artifact '{artifact_id}' is not in the registry "
                f"(known: {sorted(self._registry)})"
            )
        return base_url

    def _request(self, method: str, url: str, **kwargs) -> requests.Response:
        try:
            response = requests.request(
                method, url, timeout=REQUEST_TIMEOUT_S, **kwargs
            )
        except requests.RequestException as err:
            raise ArtifactUnavailable(f"{method} {url} failed: {err}") from err

        if 400 <= response.status_code < 500:
            raise ArtifactCommandRejected(
                f"{method} {url} rejected ({response.status_code}): {response.text}"
            )
        if response.status_code >= 500:
            raise ArtifactUnavailable(
                f"{method} {url} failed ({response.status_code}): {response.text}"
            )

        return response

    def send_command(self, artifact_id: str, command: dict) -> dict:
        """POST the command verbatim to the artifact backend.

        Returns the CommandAck body. The ack only means the command was
        written (edge-trigger); the execution outcome shows up in get_state().
        """
        url = f"{self._base_url(artifact_id)}/api/v1/artifact/command"

        self._logger.info(
            "[ArtifactGateway] Sending command",
            artifact_id=artifact_id,
            command=command,
        )

        response = self._request("POST", url, json=command)
        return response.json()

    def get_state(self, artifact_id: str) -> dict:
        """GET the artifact state (connected/stale/error_code/live_info)."""
        url = f"{self._base_url(artifact_id)}/api/v1/artifact/state"

        response = self._request("GET", url)
        return response.json()


def load_artifact_registry(
    logger: structlog.stdlib.BoundLogger, ini_path: Optional[str] = None
) -> Dict[str, str]:
    path = ini_path or os.environ.get("SYNCAI_SYSTEM_INI", DEFAULT_SYSTEM_INI)

    parser = configparser.ConfigParser()
    # Keep artifact ids as written (configparser lowercases keys by default).
    parser.optionxform = str

    if not parser.read(path) or not parser.has_section("artifacts"):
        logger.warn(
            "[ArtifactGateway] No [artifacts] registry found; "
            "ARTIFACT steps will fail until it is configured",
            path=path,
        )
        return {}

    registry = dict(parser.items("artifacts"))
    logger.info(
        "[ArtifactGateway] Loaded artifact registry",
        path=path,
        artifacts=sorted(registry),
    )
    return registry


def init_artifact_gateway(
    logger: structlog.stdlib.BoundLogger, ini_path: Optional[str] = None
) -> ArtifactGateway:
    registry = load_artifact_registry(logger=logger, ini_path=ini_path)
    return ArtifactGateway(logger=logger, registry=registry)
