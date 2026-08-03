"""Pushes a gridmap the backend just wrote into the running map_server.

Its own gateway rather than a method on ``RobotGateway``: that one is the
driver / sys_manager / nav surface (motion keys, wifi, initialpose,
NavigateToPose), and the map router has no business holding a handle that can
command the robot to move.
"""

import os
import threading
import structlog
from typing import Any, Dict, Optional, Tuple
from rclpy.node import Node
from nav2_msgs.srv import LoadMap


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
    LoadMap.Response.RESULT_INVALID_MAP_DATA: (
        "map_server could not read gridmap.pgm"
    ),
    LoadMap.Response.RESULT_INVALID_MAP_METADATA: (
        "map_server rejected gridmap.yaml"
    ),
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

        self._service_clients: Dict[str, Any] = {}
        self.register_service_clients()

    def register_service_clients(self):

        # Relative name, so it resolves under this node's robot_id namespace to
        # /<robot_id>/map_server/load_map. The extra `map_server/` segment is the
        # node name: syncai_map_server prefixes its services with get_name(),
        # unlike the flat `scan_wifi` / `set_motion_key` names elsewhere.
        #
        # This reaches the main instance only. `filter_mask_server` is a second
        # map_server serving its own load_map for the keepout mask; reloading
        # that is a different feature, not something to generalise this into.
        load_map_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="map_server/load_map",
        )

        self._service_clients.update({"load_map": load_map_client})

    def reload_map(self, yaml_path: str) -> Tuple[bool, str]:
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
            return False, "map_server/load_map is not available; is the stack running?"

        # map_io.cpp's loadMapYaml expands `~/` only to open the yaml, then
        # resolves the yaml's relative `image:` key against dirname() of the
        # string it was handed *unexpanded*. So `~/robot_ws/.../gridmap.yaml`
        # parses fine and then hands GraphicsMagick a literal `~/...` path,
        # failing as RESULT_INVALID_MAP_DATA -- which reads like a corrupt image
        # rather than a path bug. MapCatalogRepo already returns expanded
        # absolute paths; this is belt-and-braces for a future caller that reads
        # the INI's `[map] map` instead, because that value is relative.
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


def init_map_gateway(
    logger: structlog.stdlib.BoundLogger, node: Node
) -> MapGateway:
    return MapGateway(logger=logger, node=node)
