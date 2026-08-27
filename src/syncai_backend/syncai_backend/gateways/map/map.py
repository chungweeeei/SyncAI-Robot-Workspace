"""The map router's ROS surface: map_server reloads and pgo map saves.

Its own gateway rather than methods on ``RobotGateway``: that one is the
driver / sys_manager / nav surface (motion keys, wifi, initialpose,
NavigateToPose), and the map router has no business holding a handle that can
command the robot to move. Both clients here only make map files change hands.

The two clients are also never alive at the same time, which is worth knowing
before debugging either: ``map_server/load_map`` exists only in the nav (AUTO)
session, ``pgo/save_maps`` only in the mapping (MANUAL) one. A "service is not
available" from one of them usually means "wrong mode", not "broken stack".
"""

import os
import threading
import structlog
from typing import Optional
from rclpy.node import Node
from rclpy.client import Client
from nav2_msgs.srv import LoadMap
from interface.srv import SaveMaps


# LoadMap.srv carries no `message` field -- only `uint8 result` and the grid --
# so every string an operator gets for a failed reload is written here.
#
# RESULT_UNDEFINED_FAILURE is unreachable against syncai_map_server (its
# loadMapResponseFromYaml switches over exactly the four LOAD_MAP_STATUS values),
# but the srv defines it and a stock nav2 map_server on the same DDS graph would
# send it, so it is kept rather than left to the fallback.
_LOAD_MAP_MESSAGES = {
    LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST: (
        "map_server could not find the map yaml"
    ),
    LoadMap.Response.RESULT_INVALID_MAP_DATA: ("map_server could not read gridmap.pgm"),
    LoadMap.Response.RESULT_INVALID_MAP_METADATA: ("map_server rejected gridmap.yaml"),
    LoadMap.Response.RESULT_UNDEFINED_FAILURE: (
        "map_server reported an undefined failure"
    ),
}


def _wait_for_future(future, timeout: Optional[float] = None) -> bool:
    """Bridge a ROS future to the FastAPI worker thread that is waiting on it.

    The same four lines as ``gateways/robot/robot.py``, duplicated rather than
    shared so a map gateway does not have to import the robot one (and its
    NavigateToPose machinery) for a helper this size. If a third gateway needs
    it, move it to ``gateways/__init__.py`` and change all three.

    Deliberately not ``rclpy.spin_until_future_complete``: called from a thread
    that is not the executor's, that deadlocks. Here the response is delivered on
    a MultiThreadedExecutor thread while this thread parks on the Event.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout=timeout)


class MapGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):
        self._logger = logger
        self._node = node

        self._service_clients: dict[str, Client] = {}
        self.register_service_clients()

    def register_service_clients(self):

        load_map_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="map_server/load_map",
        )
ㄋ
        save_maps_client = self._node.create_client(
            srv_type=SaveMaps,
            srv_name="pgo/save_maps",
        )

        self._service_clients.update(
            {
                "load_map": load_map_client,
                "save_maps": save_maps_client
            }
        )

    def reload_map(self, yaml_path: str) -> tuple[bool, str]:
        """Make the running map_server re-read a map and re-publish it.

        ``loadMapCallback`` reads both the yaml and the .pgm off disk on every
        call -- there is no caching -- then stamps a fresh header and publishes
        onto the same transient_local publisher, which also replaces the retained
        sample so late joiners get the edit too. On failure it returns before
        publishing and leaves the previously loaded grid in place, so a rejected
        reload cannot leave the stack without a map.

        Only ever called for the *active* map. map_server holds exactly one grid
        -- the one `[map] map` named at launch -- so handing it another map's yaml
        would silently swap the running map out from under the localizer, both
        costmaps and every stored vertex.
        """
        load_map_client = self._service_clients.get("load_map")
        if not load_map_client.wait_for_service(timeout_sec=5.0):
            return False, "load_map service is not available."

        map_url = os.path.abspath(os.path.expanduser(yaml_path))

        self._logger.info("[MapGateway] Reloading map", map_url=map_url)

        future = load_map_client.call_async(LoadMap.Request(map_url=map_url))
        # The handler is synchronous: yaml parse, a GraphicsMagick decode of a
        # ~2.4 MB P5, a full pass building the OccupancyGrid, then a publish --
        # comfortably under a second for the real maps at startup. 20s is ~20x
        # headroom while still short enough that a wedged map_server does not
        # hold a FastAPI worker thread for a minute.
        if not _wait_for_future(future, timeout=20.0):
            return False, "Timeout waiting for map_server/load_map response"

        response = future.result()
        if response.result != LoadMap.Response.RESULT_SUCCESS:
            return False, _LOAD_MAP_MESSAGES.get(
                response.result, f"map_server returned result {response.result}"
            )

        return True, ""

    def save_map(self, directory: str) -> tuple[bool, str]:

        save_maps_client = self._service_clients.get("save_maps")
        if not save_maps_client.wait_for_service(timeout_sec=5.0):
            return False, (
                "save_maps service is not available"
            )

        file_path = os.path.abspath(os.path.expanduser(directory))
        self._logger.info("[MapGateway] Saving map", file_path=file_path)

        future = save_maps_client.call_async(
            SaveMaps.Request(file_path=file_path, save_patches=True)
        )

        if not _wait_for_future(future, timeout=180.0):
            return False, "Timeout waiting for pgo/save_maps response"

        response = future.result()
        return response.success, response.message


def init_map_gateway(logger: structlog.stdlib.BoundLogger, node: Node) -> MapGateway:
    return MapGateway(logger=logger, node=node)
