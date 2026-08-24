"""Tests for MapGateway: the map router's two service clients.

Same seams as test_robot_gateway.py: the node is a MagicMock and futures are
the hand-completed ``_Future`` below, so no ROS graph (or rclpy.init) is
needed. Unlike RobotGateway's single mocked node, ``create_client`` here hands
out a *distinct* mock per service name — a bare MagicMock would return the same
object for both calls, and a test could then never prove that reload_map left
``pgo/save_maps`` alone.

The timeout paths are pinned by patching the module's ``_wait_for_future``
rather than a shrunken constant: map.py inlines its 20 s / 180 s deadlines
(there is no module-level knob like robot.py's SWITCH_MODE_ACK_TIMEOUT), and
waiting out even a shrunken real deadline buys nothing the patched helper does
not. The helper itself is tested directly, with real threading, further down.
"""

import os
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
# FASTLIO2_ROS2's interface package — only present once the workspace is built
# and sourced, like syncai_common elsewhere in this suite.
pytest.importorskip("interface")

from nav2_msgs.srv import LoadMap  # noqa: E402
from interface.srv import SaveMaps  # noqa: E402

from syncai_backend.gateways.map import map as map_module  # noqa: E402
from syncai_backend.gateways.map.map import (  # noqa: E402
    MapGateway,
    _wait_for_future,
)


class _Future:
    """A future the test completes by hand; done ones fire callbacks at once."""

    def __init__(self, result=None, completed=True):
        self._result = result
        self._completed = completed
        self._callbacks = []

    def add_done_callback(self, callback):
        if self._completed:
            callback(self)
        else:
            self._callbacks.append(callback)

    def result(self):
        return self._result

    def complete(self, result):
        self._result = result
        self._completed = True
        for callback in self._callbacks:
            callback(self)


@pytest.fixture
def map_gw(logger) -> MapGateway:
    node = MagicMock()
    # One mock per srv_name (see module docstring). Keyed by name so the tests
    # can also implicitly pin that the gateway asks for the node-prefixed
    # relative names — a wrong srv_name raises KeyError right here.
    clients = {
        "map_server/load_map": MagicMock(),
        "pgo/save_maps": MagicMock(),
    }
    node.create_client.side_effect = lambda srv_type, srv_name: clients[srv_name]
    return MapGateway(logger=logger, node=node)


def _arm(map_gw, name, response=None, available=True, completed=True):
    client = map_gw._service_clients[name]
    client.wait_for_service.return_value = available
    client.call_async.return_value = _Future(result=response, completed=completed)
    return client


class TestWaitForFuture:
    """The thread bridge itself — the one piece with real concurrency in it."""

    def test_a_done_future_returns_true_immediately(self):
        # add_done_callback on a completed future fires at once, so the Event
        # is set before wait() is entered — no timeout needed at all.
        assert _wait_for_future(_Future(completed=True)) is True

    def test_an_unanswered_future_times_out_false(self):
        assert _wait_for_future(_Future(completed=False), timeout=0.01) is False

    def test_a_completion_from_another_thread_wakes_the_waiter(self):
        # The production shape: the FastAPI worker parks here while an executor
        # thread delivers the response.
        future = _Future(completed=False)
        threading.Timer(0.01, future.complete, args=(None,)).start()
        assert _wait_for_future(future, timeout=5.0) is True


class TestReloadMap:
    def test_success_sends_the_expanded_absolute_url(self, map_gw):
        client = _arm(
            map_gw,
            "load_map",
            LoadMap.Response(result=LoadMap.Response.RESULT_SUCCESS),
        )

        success, message = map_gw.reload_map(yaml_path="~/map/full/gridmap.yaml")

        assert (success, message) == (True, "")
        # The request is the observable here: map_io resolves the yaml's
        # relative `image:` key against the *unexpanded* string it is handed,
        # so a surviving `~` would fail later as a fake corrupt-image error.
        request = client.call_async.call_args[0][0]
        assert request.map_url == os.path.join(
            os.path.expanduser("~"), "map/full/gridmap.yaml"
        )
        assert os.path.isabs(request.map_url)

    def test_service_unavailable_fails_before_calling(self, map_gw):
        # In practice "wrong mode": map_server only exists in the AUTO session.
        client = _arm(map_gw, "load_map", available=False)

        success, message = map_gw.reload_map(yaml_path="/map/full/gridmap.yaml")

        assert success is False
        assert message == "load_map service is not available."
        client.call_async.assert_not_called()

    def test_timeout_reports_the_load_map_deadline(self, map_gw, monkeypatch):
        client = _arm(map_gw, "load_map", completed=False)
        monkeypatch.setattr(map_module, "_wait_for_future", lambda f, timeout: False)

        success, message = map_gw.reload_map(yaml_path="/map/full/gridmap.yaml")

        assert success is False
        assert message == "Timeout waiting for map_server/load_map response"
        client.call_async.assert_called_once()  # dispatched, then abandoned

    @pytest.mark.parametrize(
        "result_code,expected_fragment",
        [
            (LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST, "could not find the map yaml"),
            (LoadMap.Response.RESULT_INVALID_MAP_DATA, "could not read gridmap.pgm"),
            (LoadMap.Response.RESULT_INVALID_MAP_METADATA, "rejected gridmap.yaml"),
            (LoadMap.Response.RESULT_UNDEFINED_FAILURE, "undefined failure"),
        ],
    )
    def test_each_failure_code_gets_its_operator_message(
        self, map_gw, result_code, expected_fragment
    ):
        # LoadMap.srv has no message field; these strings are the whole story
        # an operator ever sees for a failed reload.
        _arm(map_gw, "load_map", LoadMap.Response(result=result_code))

        success, message = map_gw.reload_map(yaml_path="/map/full/gridmap.yaml")

        assert success is False
        assert expected_fragment in message

    def test_an_unmapped_code_falls_back_to_the_raw_result(self, map_gw):
        # A result outside the srv's named values (some other map_server
        # implementation on the graph) must still produce a usable string.
        _arm(map_gw, "load_map", SimpleNamespace(result=42))

        success, message = map_gw.reload_map(yaml_path="/map/full/gridmap.yaml")

        assert (success, message) == (False, "map_server returned result 42")


class TestSaveMap:
    def test_success_passes_pgo_answer_through(self, map_gw):
        client = _arm(
            map_gw,
            "save_maps",
            SimpleNamespace(success=True, message="MAP SAVED"),
        )

        success, message = map_gw.save_map(directory="~/map/newsite")

        assert (success, message) == (True, "MAP SAVED")
        # pgo does no expansion at all — the request must already carry the
        # absolute path, and save_patches is always on (patches/ + poses.txt
        # are what a later HBA refinement needs).
        request = client.call_async.call_args[0][0]
        assert isinstance(request, SaveMaps.Request)
        assert request.file_path == os.path.join(os.path.expanduser("~"), "map/newsite")
        assert request.save_patches is True

    def test_pgo_refusal_passes_through_verbatim(self, map_gw):
        # e.g. "NO POSES!" — a run that never banked a keyframe. The gateway
        # adds nothing; pgo's own message is the diagnostic.
        _arm(map_gw, "save_maps", SimpleNamespace(success=False, message="NO POSES!"))

        assert map_gw.save_map(directory="/map/newsite") == (False, "NO POSES!")

    def test_service_unavailable_blames_the_mode(self, map_gw):
        # pgo only runs in the mapping session, so the message must say so —
        # "service missing" alone reads as a broken stack, not the wrong mode.
        client = _arm(map_gw, "save_maps", available=False)

        success, message = map_gw.save_map(directory="/map/newsite")

        assert success is False
        assert "not available" in message
        assert "MANUAL" in message
        client.call_async.assert_not_called()

    def test_timeout_reports_the_save_maps_deadline(self, map_gw, monkeypatch):
        client = _arm(map_gw, "save_maps", completed=False)
        monkeypatch.setattr(map_module, "_wait_for_future", lambda f, timeout: False)

        success, message = map_gw.save_map(directory="/map/newsite")

        assert success is False
        assert message == "Timeout waiting for pgo/save_maps response"
        client.call_async.assert_called_once()
