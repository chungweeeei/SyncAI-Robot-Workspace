"""Tests for PathSubscriber: the planner's ``plan`` topic -> TelemetryRepo's
path slot.

What is pinned:

* The QoS asymmetry. Every other backend subscriber is best-effort; this one
  is RELIABLE depth 1, matching the planner's ``rclcpp::QoS(1)``, because a
  plan arrives every ~3 s — a dropped one leaves the operator staring at a
  route the robot has already left for the whole replan period.

* The frame gate DROPS, it never reprojects. Everything downstream treats the
  numbers as map metres; forwarding another frame's coordinates would draw a
  confidently wrong route rather than a slightly-off one. And the gate must
  not latch: its tri-state exists for edge-triggered logging only.

* The thinning contract: stride sampling to ~_MAX_PATH_POINTS, millimetre
  rounding, collapse of sub-millimetre neighbours (a duplicated point NaNs the
  frontend's ribbon normals and silently blanks the whole mesh), and the goal
  pose always surviving — it is the one point whose exact position the
  operator reads, and a non-dividing stride is exactly what would drop it.

* An empty plan is FORWARDED as the explicit clear sample, even though today's
  planner never publishes one (route clearing is TTL-based in the repo — see
  test_telemetry_repo). A future producer that does publish one must get the
  clear it obviously means.
"""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs")

import rclpy.qos  # noqa: E402

from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402

from syncai_backend.repositories.telemetry.telemetry import (  # noqa: E402
    init_telemetry_repo,
)
from syncai_backend.subscribers.path_subscriber import (  # noqa: E402
    _MAX_PATH_POINTS,
    init_path_subscriber,
)


class _FakeNode:
    """Records create_subscription calls instead of touching the DDS graph."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        sub = SimpleNamespace(
            msg_type=msg_type, topic=topic, callback=callback, qos_profile=qos_profile
        )
        self.subscriptions.append(sub)
        return sub


def _path(points, frame_id="map", sec=100, nanosec=0):
    """A nav_msgs/Path the way NavFn emits one: positions only, shared header."""
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        msg.poses.append(pose)
    return msg


@pytest.fixture
def repo(logger):
    return init_telemetry_repo(logger=logger)


@pytest.fixture
def node():
    return _FakeNode()


@pytest.fixture
def subscription(logger, node, repo):
    init_path_subscriber(logger=logger, node=node, telemetry_repo=repo)
    (sub,) = node.subscriptions
    return sub


def test_subscribes_to_the_relative_plan_topic(subscription):
    # Relative, so it inherits the robot_id namespace and reads exactly one
    # robot's planner (CLAUDE.md).
    assert subscription.topic == "plan"
    assert subscription.msg_type is Path


def test_qos_is_reliable_depth_one_matching_the_planner(subscription):
    qos = subscription.qos_profile

    # The deliberate odd-one-out among the backend subscribers: at one plan
    # per ~3 s, a drop costs the operator the whole replan period.
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.RELIABLE
    assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE
    assert qos.depth == 1


# --- the frame gate -----------------------------------------------------------


def test_a_map_frame_plan_lands_in_the_repo(subscription, repo):
    subscription.callback(
        _path([(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)], sec=100, nanosec=250_000_000)
    )

    sample = repo.get_path()
    assert sample.points == ((0.0, 0.0), (0.5, 0.5), (1.0, 1.0))
    assert sample.stamp == pytest.approx(100.25)


def test_a_plan_in_another_frame_is_dropped_not_reprojected(subscription, repo):
    # base_link coordinates plotted as map metres would be a confidently wrong
    # route on the operator's map — silence is the safer failure.
    subscription.callback(_path([(0.0, 0.0), (1.0, 1.0)], frame_id="robot01/base_link"))

    assert repo.get_path() is None


def test_plans_resume_after_a_wrong_frame(subscription, repo):
    subscription.callback(_path([(9.0, 9.0)], frame_id="robot01/odom"))
    assert repo.get_path() is None

    # The tri-state exists for edge-triggered logging only; it must not latch
    # the drop once a well-formed plan arrives.
    subscription.callback(_path([(1.0, 2.0)]))

    assert repo.get_path().points == ((1.0, 2.0),)


def test_an_empty_plan_is_forwarded_as_the_explicit_clear(subscription, repo):
    # Seed a live route so the clear is observable as a change, not a no-op.
    subscription.callback(_path([(0.0, 0.0), (1.0, 1.0)]))
    seq = repo.get_path().seq

    subscription.callback(_path([], sec=101))

    cleared = repo.get_path(after_seq=seq)
    assert cleared.points == ()
    assert cleared.stamp == pytest.approx(101.0)


# --- thinning -----------------------------------------------------------------


def test_points_are_rounded_to_millimetres(subscription, repo):
    subscription.callback(_path([(1.23456, -0.00049), (2.0004, 3.0006)]))

    # round(), so -0.00049 goes to -0.0 and 3.0006 to 3.001 — finer than the
    # 0.05 m costmap resolution the poses came from, so nothing real is lost.
    assert repo.get_path().points == ((1.235, -0.0), (2.0, 3.001))


def test_sub_millimetre_neighbours_collapse_to_one_point(subscription, repo):
    # The duplicated point is the input the frontend's ribbon builder cannot
    # survive (NaN normal blanks the mesh) — the filter here is its guarantee.
    subscription.callback(_path([(0.0, 0.0), (0.0, 0.0), (0.0004, 0.0002), (1.0, 1.0)]))

    assert repo.get_path().points == ((0.0, 0.0), (1.0, 1.0))


def test_a_single_pose_plan_is_one_point_not_two(subscription, repo):
    # The goal re-append must recognise it already kept the goal, or every
    # trivial plan would ship a zero-length segment — the exact artefact the
    # spacing filter exists to remove.
    subscription.callback(_path([(2.5, -1.5)]))

    assert repo.get_path().points == ((2.5, -1.5),)


def test_long_plans_are_thinned_by_stride_and_keep_the_goal(subscription, repo):
    # 1025 poses at NavFn's 0.05 m spacing (a ~51 m route): stride is
    # ceil(1025 / 512) = 3, so the loop keeps indices 0, 3, ..., 1023 (342
    # points) and the goal at index 1024 — which the stride would have dropped
    # — is appended on top.
    subscription.callback(_path([(i * 0.05, 0.0) for i in range(1025)]))

    points = repo.get_path().points
    assert len(points) == 343
    assert points[0] == (0.0, 0.0)
    assert points[1] == (0.15, 0.0)  # every 3rd pose, not every pose
    assert points[-1] == (51.2, 0.0)  # the goal, exactly, not the last stride hit


def test_the_goal_append_may_exceed_the_cap_by_one(subscription, repo):
    # 1024 poses: stride 2 keeps exactly _MAX_PATH_POINTS, and the goal (odd
    # index 1023, off the stride) is still appended — so the cap is soft by
    # one. Pinned as-is: the docstring's "at most _MAX_PATH_POINTS" is the
    # approximation, keeping the goal is the requirement, and tightening the
    # cap must not be done by dropping it.
    subscription.callback(_path([(i * 0.05, 0.0) for i in range(1024)]))

    points = repo.get_path().points
    assert len(points) == _MAX_PATH_POINTS + 1
    assert points[-1] == (51.15, 0.0)
