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

# FASTLIO2_ROS2's interface package — `interface` really is its name. Declared
# in package.xml so colcon orders the build; there is no pip/rosdep fallback.
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

        # Relative name again, with the pgo node's own prefix:
        # /<robot_id>/pgo/save_maps. Served only while a mapping session is up.
        save_maps_client = self._node.create_client(
            srv_type=SaveMaps,
            srv_name="pgo/save_maps",
        )

        self._service_clients.update(
            {"load_map": load_map_client, "save_maps": save_maps_client}
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

    def save_map(self, directory: str) -> tuple[bool, str]:
        """Ask pgo to serialise its accumulated keyframes into ``directory``.

        The directory must already exist: ``saveMapsCB`` answers
        ``"<path> IS NOT EXISTS!"`` otherwise — it only ever creates the
        ``patches/`` subdirectory itself. The caller (MapCatalogRepo's
        ``create_map_dir``) owns making it, so this method never touches the
        filesystem beyond the abspath below.

        ``save_patches=True`` always: ``patches/`` + ``poses.txt`` are what a
        later HBA refinement or ``RefineMap`` needs, they cost disk only, and
        every existing map directory on this robot carries them.

        A failure here is *usually* "wrong mode": pgo only runs in the mapping
        session, so in AUTO the wait_for_service below is what fails. The other
        common answer is ``"NO POSES!"`` — a mapping run that has not moved far
        enough to bank a single keyframe (10 deg / 0.5 m thresholds).
        """
        save_maps_client = self._service_clients.get("save_maps")
        if not save_maps_client.wait_for_service(timeout_sec=5.0):
            return False, (
                "save_maps service is not available — pgo only runs in "
                "mapping (MANUAL) mode."
            )

        # Same trap as reload_map's map_url: pgo does no expansion at all, it
        # hands the string to std::filesystem, so a `~` or a path relative to
        # the backend's cwd would land somewhere surprising or nowhere.
        file_path = os.path.abspath(os.path.expanduser(directory))

        self._logger.info("[MapGateway] Saving map", file_path=file_path)

        future = save_maps_client.call_async(
            SaveMaps.Request(file_path=file_path, save_patches=True)
        )
        # Far above reload_map's 20 s because the handler genuinely works for
        # its living: it merges every keyframe cloud, voxel-filters the result
        # and writes a ~20 MB map.pcd plus one small .pcd per keyframe — on
        # this Jetson, tens of seconds for a large site. A timeout that fired
        # mid-write would report failure for a save that then completes, so it
        # errs long; the FastAPI handler parking on this runs on the worker
        # thread pool, not the event loop.
        if not _wait_for_future(future, timeout=180.0):
            return False, "Timeout waiting for pgo/save_maps response"

        response = future.result()
        return response.success, response.message


def init_map_gateway(logger: structlog.stdlib.BoundLogger, node: Node) -> MapGateway:
    return MapGateway(logger=logger, node=node)
