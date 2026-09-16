"""The map router's ROS surface: map_server reloads, localizer swaps, pgo saves.

Its own gateway rather than methods on ``RobotGateway``: that one is the
driver / sys_manager / nav surface (motion keys, wifi, initialpose,
NavigateToPose), and the map router has no business holding a handle that can
command the robot to move. Everything here only makes map files change hands --
including the ``initialpose`` publisher, which re-seeds *where the robot thinks
it is* and never asks it to go anywhere. That publisher is a second one on a
topic ``RobotGateway`` also publishes to, which is fine in ROS and is the lesser
evil: the alternative is handing the map router the gateway that owns
NavigateToPose.

The clients split by session, which is worth knowing before debugging any of
them: ``map_server/load_map``, ``relocalize`` and ``relocalize_check``
exist only in the nav (AUTO) session,
``pgo/save_maps`` only in the mapping (MANUAL) one. A "service is not available"
from one of them usually means "wrong mode", not "broken stack" -- which is
exactly what ``POST /api/v1/maps/{name}/activate`` turns into its
``stack_not_ready`` refusal.
"""

import math
import os
import threading
import time
import structlog
from typing import Optional
from rclpy.node import Node
from rclpy.client import Client
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Header
from nav2_msgs.srv import LoadMap
from interface.srv import IsValid, Relocalize, SaveMaps


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

        # Same topic and QoS as RobotGateway's: volatile, depth 5. See the
        # module docstring for why this gateway has its own.
        self._initial_pose_pub = self._node.create_publisher(
            PoseWithCovarianceStamped, "initialpose", QoSProfile(depth=5)
        )

    def register_service_clients(self):

        load_map_client = self._node.create_client(
            srv_type=LoadMap,
            srv_name="map_server/load_map",
        )

        save_maps_client = self._node.create_client(
            srv_type=SaveMaps,
            srv_name="pgo/save_maps",
        )

        # Bare names, unlike load_map's. The localizer declares these with
        # plain `create_service("relocalize", ...)` (localizer_node.cpp:99-108),
        # so they resolve against its *namespace* and land at
        # /<robot_id>/relocalize -- while map_server builds its own
        # `service_prefix + "load_map"`, which is why that one carries the node
        # name and these do not. Writing `localizer/relocalize` here, on the
        # reasonable-looking assumption that it matched the node, cost a
        # stack_not_ready refusal against a perfectly healthy stack: a stale
        # entry under that name can even linger in `ros2 service list` after a
        # participant dies, so the list is not the check -- `ros2 node info
        # /<robot_id>/localizer_node` is.
        relocalize_client = self._node.create_client(
            srv_type=Relocalize,
            srv_name="relocalize",
        )

        relocalize_check_client = self._node.create_client(
            srv_type=IsValid,
            srv_name="relocalize_check",
        )

        self._service_clients.update(
            {
                "load_map": load_map_client,
                "save_maps": save_maps_client,
                "relocalize": relocalize_client,
                "relocalize_check": relocalize_check_client,
            }
        )

    def nav_services_ready(self, timeout_sec: float = 2.0) -> bool:
        """Are the AUTO-session map services discoverable right now?

        The precondition for a map switch, and the honest way to ask "is the nav
        stack up". Deliberately not read off the cached RobotState's mode:
        RobotRepo's write is gated on `localization_valid`, so a robot that has
        lost localization has no cached state and therefore no mode -- and that
        is exactly the robot whose operator most wants to switch maps.
        """
        return all(
            self._service_clients[key].wait_for_service(timeout_sec=timeout_sec)
            for key in ("relocalize", "load_map")
        )

    def swap_localizer_map(
        self, pcd_path: str, x: float, y: float, yaw: float
    ) -> tuple[bool, str]:
        """Point the running localizer at another map's cloud and re-seed it.

        Two calls, and the second is not optional.

        `relocalize` swaps the map: `relocCB` checks the path exists, hands it to
        `ICPLocalizer::loadMap` -- which builds into local buffers and swaps only
        as its last statement, so a failed load leaves the old map serving rather
        than leaving the localizer with nothing -- then stores an initial guess
        and republishes the latched map cloud. Note what its `success` does *not*
        mean: registration has not run. It happens on the node's 5 Hz timer
        afterwards, with no deadline and no give-up, and the only surface that
        ever reports it is `relocalize_check`.

        The guess handed to `relocalize` here is deliberately transient. `relocCB`
        takes the request's raw 6-DOF, bypassing `applyPlanarGuess` -- the path
        `initialpose` goes through, which re-fills roll/pitch/z from the current
        estimate. That matters because the lidar is mounted tilted: map_T_body
        always carries ~15 degrees of pitch, and a flat roll=pitch=0 guess never
        passes `rough_max_corr_dist`, so the localizer freezes retrying it
        forever (the failure `applyPlanarGuess` was written to prevent). So the
        zeros below only satisfy `relocCB`'s demand for *a* guess; the
        `initialpose` publish immediately overwrites it with a tilt-correct one,
        which the timer picks up on its next 200 ms cycle. Dropping the second
        call reintroduces the freeze.
        """
        relocalize_client = self._service_clients.get("relocalize")
        if not relocalize_client.wait_for_service(timeout_sec=5.0):
            return False, "relocalize service is not available."

        map_path = os.path.abspath(os.path.expanduser(pcd_path))

        self._logger.info("[MapGateway] Relocalizing onto map cloud", pcd=map_path)

        future = relocalize_client.call_async(
            Relocalize.Request(
                pcd_path=map_path, x=float(x), y=float(y), z=0.0,
                yaw=float(yaw), pitch=0.0, roll=0.0,
            )
        )
        # The handler reads a ~100-200 MB PCD off disk and voxel-downsamples it
        # twice before returning. 60s is generous for the conference-hall clouds
        # while still bounded, since this holds a FastAPI worker thread.
        if not _wait_for_future(future, timeout=60.0):
            return False, "Timeout waiting for relocalize response"

        response = future.result()
        if not response.success:
            return False, f"localizer refused the map: {response.message}"

        self._seed_initial_pose(x=x, y=y, yaw=yaw)
        return True, ""

    def _seed_initial_pose(self, x: float, y: float, yaw: float) -> None:
        """Publish `initialpose` so the guess goes through `applyPlanarGuess`.

        Covariance stays zero: the localizer reads x/y/yaw only, and a map switch
        has no meaningful uncertainty to report. A missing subscriber is logged
        for the same reason RobotGateway logs it -- an initialpose nobody
        receives looks exactly like one that was received and ignored.
        """
        if self._initial_pose_pub.get_subscription_count() == 0:
            self._logger.warning(
                "[MapGateway] Publishing initialpose with no subscriber; "
                "is the localizer running?"
            )

        self._logger.info("[MapGateway] Seeding initial pose", x=x, y=y, yaw=yaw)

        msg = PoseWithCovarianceStamped(
            header=Header(
                frame_id="map", stamp=self._node.get_clock().now().to_msg()
            )
        )
        msg.pose.pose = Pose(
            position=Point(x=float(x), y=float(y), z=0.0),
            orientation=Quaternion(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
            ),
        )
        self._initial_pose_pub.publish(msg)

    def localization_converged(self, timeout_s: float = 3.0) -> Optional[bool]:
        """Has the localizer's ICP actually converged? None if it cannot be asked.

        Polls `relocalize_check` (`code=0`, the branch that reports the real
        `localize_success` rather than the unconditional true). This is the only
        surface in the entire stack that answers the question: the localizer
        publishes no status topic, and `RobotState.localization_valid` is
        TF-presence only -- the localizer broadcasts map->odom unconditionally,
        with an identity offset, from the first odom sample, so a localizer that
        has never converged still reports `localization_valid: true` all the way
        to the console.

        Best effort by construction. A false here means "not yet", not "never":
        registration retries at 5 Hz indefinitely, so a caller that waits longer
        would sometimes get a true. The short budget exists because this runs
        inside a REST handler, and the operator's recourse -- setting an initial
        pose from the dashboard -- is the same either way.
        """
        check_client = self._service_clients.get("relocalize_check")
        if not check_client.wait_for_service(timeout_sec=1.0):
            return None

        deadline = time.monotonic() + timeout_s
        while True:
            future = check_client.call_async(IsValid.Request(code=0))
            remaining = max(0.5, deadline - time.monotonic())
            if not _wait_for_future(future, timeout=remaining):
                return None
            if future.result().valid:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.25)

    def reload_map(self, yaml_path: str) -> tuple[bool, str]:
        """Make the running map_server re-read a map and re-publish it.

        ``loadMapCallback`` reads both the yaml and the .pgm off disk on every
        call -- there is no caching -- then stamps a fresh header and publishes
        onto the same transient_local publisher, which also replaces the retained
        sample so late joiners get the edit too. On failure it returns before
        publishing and leaves the previously loaded grid in place, so a rejected
        reload cannot leave the stack without a map.

        map_server holds exactly one grid -- the one `[map] map` named at launch
        -- so this call *is* the running map, for the localizer, both costmaps and
        every stored vertex. That is why it was for a long time only ever called
        for the active map: handing it another map's yaml on its own swaps the
        grid out from under a localizer still registering against the old cloud.

        `POST /api/v1/maps/{name}/activate` is the one caller that legitimately
        passes a different map, because it does not do it on its own -- it pairs
        this with `swap_localizer_map` and the INI write, so the grid, the point
        cloud and `active_name()` move together. The pairing is the precondition,
        not the map's identity: any future caller that reaches for this with a
        non-active name and no matching localizer swap is the bug this paragraph
        used to forbid outright.
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
            # Name the mode: pgo runs only in the mapping (MANUAL) session, and
            # a bare "service is not available" reads as a broken stack rather
            # than the wrong one.
            return False, (
                "pgo/save_maps is not available — saving a map needs the robot "
                "in MANUAL (mapping) mode."
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
