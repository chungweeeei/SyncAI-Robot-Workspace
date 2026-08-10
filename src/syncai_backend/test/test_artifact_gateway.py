"""Tests for ArtifactGateway: the REST bridge the ARTIFACT activities ride on.

Mocked at ``requests.request`` — the gateway's only I/O — following the
CoreManager gateway-test pattern (mock the transport seam, pin success /
rejection / unavailability per method). The error taxonomy matters more than
the happy path here: the Temporal activity retries ``ArtifactUnavailable`` but
treats ``ArtifactCommandRejected`` and ``UnknownArtifactError`` as
non-retryable, so a miscategorised status code would either retry an
edge-triggered command or give up on a transient outage.
"""

from types import SimpleNamespace
from unittest.mock import patch

import pytest
import requests

from syncai_backend.gateways.artifact.artifact import (
    ArtifactCommandRejected,
    ArtifactGateway,
    ArtifactUnavailable,
    UnknownArtifactError,
    init_artifact_gateway,
    load_artifact_registry,
)


REQUEST = "syncai_backend.gateways.artifact.artifact.requests.request"


def _response(status_code: int, body: dict | None = None) -> SimpleNamespace:
    return SimpleNamespace(
        status_code=status_code,
        text="err" if body is None else str(body),
        json=lambda: body or {},
    )


@pytest.fixture
def artifact_gw(logger) -> ArtifactGateway:
    # Trailing slash on purpose: the constructor must strip it, or every URL
    # would carry a double slash.
    return ArtifactGateway(
        logger=logger, registry={"conveyor01": "http://10.0.0.33:8001/"}
    )


class TestArtifactGateway:
    def test_send_command_posts_verbatim_and_returns_the_ack(self, artifact_gw):
        with patch(REQUEST, return_value=_response(200, {"accepted": True})) as req:
            ack = artifact_gw.send_command(
                "conveyor01", {"action": "pickup", "robot": "any", "box": 0}
            )

        assert ack == {"accepted": True}
        args, kwargs = req.call_args
        assert args == ("POST", "http://10.0.0.33:8001/api/v1/artifact/command")
        # Forwarded verbatim — the artifact backend re-validates it.
        assert kwargs["json"] == {"action": "pickup", "robot": "any", "box": 0}

    def test_get_state_hits_the_state_endpoint(self, artifact_gw):
        state = {"connected": True, "error_code": 0, "live_info": {"phase": "belt"}}
        with patch(REQUEST, return_value=_response(200, state)) as req:
            assert artifact_gw.get_state("conveyor01") == state

        assert req.call_args[0] == ("GET", "http://10.0.0.33:8001/api/v1/artifact/state")

    def test_unknown_artifact_names_the_known_ones(self, artifact_gw):
        # Non-retryable in the activity, so the message is the operator's only
        # lead — it must say what WOULD have worked.
        with pytest.raises(UnknownArtifactError, match="conveyor01"):
            artifact_gw.send_command("conveyor99", {"action": "pickup"})

    def test_a_4xx_is_a_rejection(self, artifact_gw):
        with patch(REQUEST, return_value=_response(422)):
            with pytest.raises(ArtifactCommandRejected):
                artifact_gw.send_command("conveyor01", {"action": "pickup"})

    def test_a_5xx_is_unavailability(self, artifact_gw):
        with patch(REQUEST, return_value=_response(503)):
            with pytest.raises(ArtifactUnavailable):
                artifact_gw.get_state("conveyor01")

    def test_a_transport_error_is_unavailability(self, artifact_gw):
        with patch(REQUEST, side_effect=requests.ConnectionError("refused")):
            with pytest.raises(ArtifactUnavailable):
                artifact_gw.get_state("conveyor01")


class TestArtifactRegistry:
    def test_loads_ids_with_case_preserved(self, logger, tmp_path):
        # configparser lowercases keys by default; artifact ids are case-
        # sensitive registry keys, so the loader overrides optionxform.
        ini = tmp_path / "system.ini"
        ini.write_text(
            "[artifacts]\n"
            "conveyor01 = http://10.0.0.33:8001\n"
            "Dock-A = http://10.0.0.34:8001/\n"
        )
        registry = load_artifact_registry(logger=logger, ini_path=str(ini))
        assert registry == {
            "conveyor01": "http://10.0.0.33:8001",
            "Dock-A": "http://10.0.0.34:8001/",
        }

    def test_missing_ini_yields_an_empty_registry(self, logger, tmp_path):
        # ARTIFACT steps will fail later, but the backend must still boot.
        assert load_artifact_registry(logger, str(tmp_path / "absent.ini")) == {}

    def test_missing_section_yields_an_empty_registry(self, logger, tmp_path):
        ini = tmp_path / "system.ini"
        ini.write_text("[system]\nrobot_id = robot01\n")
        assert load_artifact_registry(logger, str(ini)) == {}

    def test_init_wires_the_registry_through(self, logger, tmp_path):
        ini = tmp_path / "system.ini"
        ini.write_text("[artifacts]\nconveyor01 = http://10.0.0.33:8001/\n")
        gw = init_artifact_gateway(logger=logger, ini_path=str(ini))
        # The base URL is normalised at construction (trailing slash stripped).
        assert gw._base_url("conveyor01") == "http://10.0.0.33:8001"
