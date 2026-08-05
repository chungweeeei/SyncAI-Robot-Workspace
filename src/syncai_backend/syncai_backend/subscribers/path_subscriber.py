import structlog

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Path

from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo


# Frame the planner's global costmap works in. Deliberately un-namespaced:
# `global_frame` is the one frame parameter the nav launch files never rewrite
# per robot (see syncai_planner/launch/planner_server.launch.py), because a
# single shared map frame is the point of it.
_MAP_FRAME = "map"

# Upper bound on the points forwarded to the viewport. NavFn emits a pose per
# costmap cell (0.05 m), so a 40 m route is ~800 poses and a warehouse crossing
# more; at 512 the band the frontend draws is already smoother than the screen
# can show, and the JSON stays around 8 kB.
_MAX_PATH_POINTS = 512

# Consecutive poses closer together than this (metres) are treated as the same
# point. Zero-length segments are what the frontend's ribbon builder cannot
# survive — it derives a surface normal from the tangent between neighbours, and
# a duplicated point makes that normal NaN, which silently blanks the whole mesh.
# Filtering here rather than there keeps the wire format free of samples no
# consumer could use.
_MIN_POINT_SPACING_M = 1e-3


class PathSubscriber:
    """Feeds TelemetryRepo's path slot, and therefore the telemetry WebSocket,
    from the planner's ``plan`` topic.

    Kept apart from TelemetrySubscriber even though it writes the same repo: that
    one exists to carry pose and joints at the rate a gait actually moves at,
    composing them through TF, whereas a global route arrives about once every
    3 s, needs no transform (it is published in the map frame already) and wants
    the opposite QoS. The only thing the two share is the socket they end up on.

    Not ``received_global_plan`` from the controller: that is the pruned slice of
    the route left in front of the robot, transformed into ``<robot_id>/base_link``
    and truncated to the local costmap. The viewport wants the whole remaining
    route in map coordinates, which is exactly what ``plan`` is.

    Two properties of the publisher shape what this can promise
    (syncai_planner/src/planner_server.cpp):

    1. ``publishPlan`` skips the publish entirely when nothing is subscribed, and
       the QoS is VOLATILE, so there is no last-value replay. After a backend
       restart mid-run the viewport has no route until the BT replans — up to
       ~3 s of blank, which is why nothing here tries to synthesise one.
    2. It publishes only on success. Arrival, cancellation and abort are all
       indistinguishable silence, which is why clearing the route is TTL-based in
       the repo rather than event-based here.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger, telemetry_repo: TelemetryRepo):
        self._logger = logger
        self._repo = telemetry_repo

        # Edge-triggered logging for the frame check below, same tri-state
        # pattern as TelemetrySubscriber._tf_available: log once when paths start
        # being dropped and once when they recover, not once per plan.
        self._frame_ok = None

    def register(self, node: Node):
        # RELIABLE, matching the publisher's rclcpp::QoS(1) exactly, rather than
        # the BEST_EFFORT the other subscribers use. Those read 20 Hz feeds where
        # the next sample is 50 ms away; a plan arrives every ~3 s, so a dropped
        # one leaves the operator looking at a route the robot has already left.
        node.create_subscription(
            msg_type=Path,
            topic="plan",  # relative, so it inherits the robot_id namespace
            callback=self._plan_cb,
            qos_profile=QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            ),
        )

    def _plan_cb(self, msg: Path):
        frame = msg.header.frame_id
        if frame != _MAP_FRAME:
            # Dropped, not reprojected. Everything downstream — the wire format,
            # the frontend's ground band, the map raster it is drawn on — treats
            # these numbers as map metres, so plotting another frame's
            # coordinates would draw a confidently wrong route rather than a
            # slightly-off one.
            if self._frame_ok is not False:
                self._logger.warning(
                    "planned path dropping: unexpected frame (expected the "
                    "global costmap's map frame)",
                    frame=frame,
                    expected=_MAP_FRAME,
                )
                self._frame_ok = False
            return

        if self._frame_ok is not True:
            self._logger.info("planned path streaming", frame=frame)
            self._frame_ok = True

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Defensive: the planner only publishes on success, so an empty plan is
        # not something it produces today. Forwarding it as the clear sample
        # costs nothing and means a future producer that does publish one gets
        # the behaviour it would expect.
        if not msg.poses:
            self._repo.update_path(points=(), stamp=stamp)
            return

        self._repo.update_path(points=self._thin(msg), stamp=stamp)

    def _thin(self, msg: Path) -> tuple:
        """Reduce a plan to at most _MAX_PATH_POINTS deduplicated (x, y) pairs."""
        # Stride sampling rather than a distance- or curvature-aware simplifier:
        # NavFn's output is already uniformly spaced along the route, so keeping
        # every nth pose loses nothing a 0.12 m wide band on the floor could
        # show, and it costs one pass.
        stride = max(1, -(-len(msg.poses) // _MAX_PATH_POINTS))  # ceil division

        points = []
        for i in range(0, len(msg.poses), stride):
            p = msg.poses[i].pose.position
            # Rounded to millimetres purely to keep the JSON small — three
            # decimals is far finer than the costmap resolution the poses came
            # from, so nothing is lost.
            xy = (round(p.x, 3), round(p.y, 3))
            if points and self._too_close(points[-1], xy):
                continue
            points.append(xy)

        # The goal end of the route is the one point whose exact position the
        # operator is reading, and a stride that does not divide the length
        # evenly is exactly what would drop it.
        last = msg.poses[-1].pose.position
        goal = (round(last.x, 3), round(last.y, 3))
        if not points or not self._too_close(points[-1], goal):
            points.append(goal)

        return tuple(points)

    @staticmethod
    def _too_close(a: tuple, b: tuple) -> bool:
        return (
            abs(a[0] - b[0]) < _MIN_POINT_SPACING_M
            and abs(a[1] - b[1]) < _MIN_POINT_SPACING_M
        )


def init_path_subscriber(
    logger: structlog.stdlib.BoundLogger, node: Node, telemetry_repo: TelemetryRepo
) -> PathSubscriber:
    subscriber = PathSubscriber(logger=logger, telemetry_repo=telemetry_repo)
    subscriber.register(node=node)
    return subscriber
