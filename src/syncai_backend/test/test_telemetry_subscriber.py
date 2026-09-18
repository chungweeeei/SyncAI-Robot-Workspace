"""Tests for TelemetrySubscriber: raw odom / lio_odom / motor_states -> TelemetryRepo.

What is pinned:

* The planar pose composition, map_T_base = map_T_odom * odom_T_base, done as
  (x, y, yaw) math against a known map->odom transform. Everything upstream is
  2D-projected by lio_bridge, so this arithmetic is exact — and a sign slip in
  the rotation terms would draw the robot mirrored on the operator's map while
  every individual number still looked plausible.

* The no-TF behaviour: until the localizer is relocalized there is no
  map->odom, and the callback must DROP the sample and return, not raise — an
  exception here escapes into the executor at 20 Hz. The lookup asks for the
  latest transform (Time()), never the message stamp, so the odom feed cannot
  be starved by future-extrapolation errors.

* The motor_states unit trap: ``MotorStates.timestamp`` is NANOSECONDS on this
  topic (the nested copy in RobotState carries seconds — the msg definition
  spells the difference out). The repo's wire format uses seconds, so the 1e-9
  scale here is load-bearing and has a documented temptation to "fix".

* The mapping-mode pose path (``pointlio/lio_odom``), which is the only pose
  the console gets during a mapping run: the full 3x3 composition through the
  tilted lidar mount, and the rule that a live ``odom`` feed silences it —
  both topics are published in AUTO and the two must never take turns.

* All subscriptions use RELATIVE topic names (namespace inheritance —
  CLAUDE.md) and best-effort QoS: motor_states' publisher is SensorDataQoS and
  a reliable subscriber would simply never match it.
"""

import math
from types import SimpleNamespace

import pytest

pytest.importorskip("syncai_common")
pytest.importorskip("rclpy")
pytest.importorskip("nav_msgs")
pytest.importorskip("tf2_ros")

import rclpy.qos  # noqa: E402
import rclpy.time  # noqa: E402

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from tf2_ros import TransformException  # noqa: E402

from syncai_common.msg import MotorState, MotorStates  # noqa: E402

from syncai_backend.helpers.pointcloud import (  # noqa: E402
    quat_to_rotation_matrix,
)
from syncai_backend.repositories.telemetry.telemetry import (  # noqa: E402
    init_telemetry_repo,
)
from syncai_backend.subscribers.telemetry_subscriber import (  # noqa: E402
    init_telemetry_subscriber,
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


class _FakeTfBuffer:
    """The one method the subscriber calls, with a switchable answer.

    ``transform=None`` plays the not-yet-relocalized buffer by raising
    TransformException — the exact exception type a real Buffer raises when
    map->odom has never been published, and the only one the callback catches.

    ``transforms`` answers per (target, source) pair, for the mapping path:
    that one composes two different lookups, so a single answer would hide a
    swapped pair. A pair that is not in the dict raises, like an absent branch
    of the real tree.
    """

    def __init__(self, transform=None, transforms=None):
        self.transform = transform
        self.transforms = transforms
        self.lookups = []

    def lookup_transform(self, target_frame, source_frame, time):
        self.lookups.append((target_frame, source_frame, time))
        if self.transforms is not None:
            try:
                return self.transforms[(target_frame, source_frame)]
            except KeyError:
                raise TransformException(
                    f"{target_frame}->{source_frame} not in the buffer"
                ) from None
        if self.transform is None:
            raise TransformException("map->odom not in the buffer")
        return self.transform


def _quat_from_matrix(rot):
    """(x, y, z, w) of a rotation matrix — Shepperd's w-branch, which is all
    the test poses need (none of them is near a 180 deg turn)."""
    w = math.sqrt(max(0.0, 1.0 + rot[0][0] + rot[1][1] + rot[2][2])) / 2.0
    s = 4.0 * w
    return (
        (rot[2][1] - rot[1][2]) / s,
        (rot[0][2] - rot[2][0]) / s,
        (rot[1][0] - rot[0][1]) / s,
        w,
    )


def _quat_for_yaw(yaw):
    """Pure-z quaternion — the only kind the 2D-projected LIO chain emits."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _odom(x, y, z=0.0, yaw=0.0, frame_id="robot01/odom", sec=100, nanosec=0):
    msg = Odometry()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = float(z)
    qx, qy, qz, qw = _quat_for_yaw(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def _map_odom_tf(x=0.0, y=0.0, z=0.0, yaw=0.0):
    tf = TransformStamped()
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.translation.z = float(z)
    qx, qy, qz, qw = _quat_for_yaw(yaw)
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return tf


@pytest.fixture
def repo(logger):
    return init_telemetry_repo(logger=logger)


@pytest.fixture
def node():
    return _FakeNode()


@pytest.fixture
def tf_buffer():
    """Starts empty (pre-relocalization); tests set ``.transform`` to recover."""
    return _FakeTfBuffer()


@pytest.fixture
def subs(logger, node, repo, tf_buffer):
    """The two subscriptions, keyed by topic so tests read like the graph."""
    init_telemetry_subscriber(
        logger=logger, node=node, telemetry_repo=repo, tf_buffer=tf_buffer
    )
    return {sub.topic: sub for sub in node.subscriptions}


def test_subscribes_to_the_relative_pose_and_motor_states_topics(subs):
    # Relative names, so all three inherit the robot_id namespace (CLAUDE.md).
    assert set(subs) == {"odom", "pointlio/lio_odom", "motor_states"}
    assert subs["odom"].msg_type is Odometry
    # The mapping-mode pose feed: same type, different frames and different
    # arithmetic (see the composition tests below).
    assert subs["pointlio/lio_odom"].msg_type is Odometry
    assert subs["motor_states"].msg_type is MotorStates


def test_qos_is_best_effort_keep_last_on_every_feed(subs):
    for topic in ("odom", "pointlio/lio_odom", "motor_states"):
        qos = subs[topic].qos_profile
        # motor_states' publisher is SensorDataQoS: a reliable subscriber here
        # would not be QoS-compatible and would simply never receive a sample.
        assert qos.reliability == rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE
        assert qos.depth == 5


# --- pose: odom composed with the map->odom correction ------------------------


def test_identity_correction_passes_the_odom_pose_through(subs, repo, tf_buffer):
    tf_buffer.transform = _map_odom_tf()  # map == odom, i.e. just relocalized

    subs["odom"].callback(
        _odom(x=1.5, y=-2.5, z=0.25, yaw=math.pi / 2, sec=100, nanosec=500_000_000)
    )

    pose = repo.get_pose()
    assert (pose.x, pose.y, pose.z) == pytest.approx((1.5, -2.5, 0.25))
    assert pose.yaw_deg == pytest.approx(90.0)
    # Sub-second resolution is the whole reason this bypasses robot_state.
    assert pose.stamp == pytest.approx(100.5)


def test_map_odom_correction_is_composed_as_planar_math(subs, repo, tf_buffer):
    """map_T_base = map_T_odom * odom_T_base, with a rotating correction.

    map->odom translates (2, 3) and rotates +90 deg, so the odom-frame point
    (1, 0) must land at (2, 4) — the x offset rotated onto map's y axis. A
    dropped sin term or a transposed rotation gives (3, 3) or (2, 2), which is
    what this transform is chosen to distinguish.
    """
    tf_buffer.transform = _map_odom_tf(x=2.0, y=3.0, z=0.1, yaw=math.pi / 2)

    subs["odom"].callback(_odom(x=1.0, y=0.0, z=0.05, yaw=math.pi / 4))

    pose = repo.get_pose()
    assert (pose.x, pose.y) == pytest.approx((2.0, 4.0))
    # z is a straight sum: both frames are gravity-aligned after the 2D
    # projection, so there is no rotation to put z through.
    assert pose.z == pytest.approx(0.15)
    assert pose.yaw_deg == pytest.approx(135.0)


def test_composed_yaw_is_not_normalized(subs, repo, tf_buffer):
    # 135 + 90 = 225, and 225 is what ships: the subscriber adds the two yaws
    # and converts, no wrap into [-180, 180]. The frontend feeds it to a
    # rotation matrix, which is periodic anyway — pinned so a wrap is added
    # deliberately (with the consumer checked), not in passing.
    tf_buffer.transform = _map_odom_tf(yaw=3.0 * math.pi / 4.0)

    subs["odom"].callback(_odom(x=0.0, y=0.0, yaw=math.pi / 2))

    assert repo.get_pose().yaw_deg == pytest.approx(225.0)


def test_lookup_asks_for_the_latest_map_to_odom(subs, tf_buffer):
    tf_buffer.transform = _map_odom_tf()

    subs["odom"].callback(_odom(x=0.0, y=0.0, frame_id="robot01/odom"))

    # Source frame comes from the message header (namespaced by lio_bridge),
    # target is the shared un-namespaced map frame, and the time is Time() —
    # latest available, so the 20 Hz feed never trips future-extrapolation.
    assert tf_buffer.lookups == [("map", "robot01/odom", rclpy.time.Time())]


def test_a_missing_correction_drops_the_sample_without_raising(subs, repo, tf_buffer):
    # tf_buffer starts empty: the pre-relocalization state in nav, and the
    # permanent state of a mapping run. Raising here would take the executor
    # callback down 20 times a second.
    subs["odom"].callback(_odom(x=1.0, y=1.0))

    assert repo.get_pose() is None


def test_pose_streaming_resumes_when_the_correction_appears(subs, repo, tf_buffer):
    subs["odom"].callback(_odom(x=1.0, y=1.0))
    assert repo.get_pose() is None

    # The localizer relocalizes: lio_bridge starts broadcasting map->odom and
    # the very next sample must flow — the drop state must not latch.
    tf_buffer.transform = _map_odom_tf()
    subs["odom"].callback(_odom(x=1.0, y=1.0))

    assert repo.get_pose().x == pytest.approx(1.0)


# --- pose: the mapping run's lio_odom, composed through the lidar mount -------


def _tf(x=0.0, y=0.0, z=0.0, quat=(0.0, 0.0, 0.0, 1.0)):
    tf = TransformStamped()
    tf.transform.translation.x = float(x)
    tf.transform.translation.y = float(y)
    tf.transform.translation.z = float(z)
    tf.transform.rotation.x, tf.transform.rotation.y = quat[0], quat[1]
    tf.transform.rotation.z, tf.transform.rotation.w = quat[2], quat[3]
    return tf


def _pitch_quat(pitch):
    """Pure-y quaternion — the shape of the G23's tilted lidar mount."""
    return (0.0, math.sin(pitch / 2.0), 0.0, math.cos(pitch / 2.0))


def _lio_odom(x, y, z=0.0, quat=(0.0, 0.0, 0.0, 1.0), sec=100, nanosec=0):
    msg = _odom(x=x, y=y, z=z, sec=sec, nanosec=nanosec)
    msg.header.frame_id = "robot01/pointlio_odom"
    msg.child_frame_id = "robot01/pointlio_body"
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = quat[0], quat[1]
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quat[2], quat[3]
    return msg


MOUNT_PITCH = math.radians(12.0)
MOUNT = {
    # base_link -> lidar_top, the URDF mount: forward, up, and pitched down.
    ("robot01/base_link", "robot01/lidar_top"): _tf(
        x=0.2, z=0.4, quat=_pitch_quat(MOUNT_PITCH)
    ),
    # map -> pointlio_odom, which pgo broadcasts during a mapping run.
    ("map", "robot01/pointlio_odom"): _tf(),
}


@pytest.fixture
def mapping_tf():
    """A mapping run's TF tree: pgo's map->odom plus bringup's mount."""
    return _FakeTfBuffer(transforms=dict(MOUNT))


@pytest.fixture
def mapping_subs(logger, node, repo, mapping_tf):
    init_telemetry_subscriber(
        logger=logger, node=node, telemetry_repo=repo, tf_buffer=mapping_tf
    )
    return {sub.topic: sub for sub in node.subscriptions}


def test_lio_pose_is_reported_at_base_link_not_at_the_lidar(mapping_subs, repo):
    """The mount extrinsic is taken back out, all six degrees of it.

    The robot stands at the map origin, so pointlio — whose body frame IS the
    lidar — reports the mount itself: 0.2 m forward, 0.4 m up, pitched down 12
    deg. Everything must come back out: (0, 0, 0) with zero heading. Feeding
    the lidar pose through instead would draw the robot a fifth of a metre
    ahead of itself and floating, which on a mapping run reads as drift.
    """
    mapping_subs["pointlio/lio_odom"].callback(
        _lio_odom(x=0.2, y=0.0, z=0.4, quat=_pitch_quat(MOUNT_PITCH))
    )

    pose = repo.get_pose()
    assert (pose.x, pose.y, pose.z) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert pose.yaw_deg == pytest.approx(0.0, abs=1e-9)


def test_lio_yaw_survives_the_tilt(mapping_subs, repo):
    """Heading comes out of the composed rotation, not out of added yaws.

    The robot has turned 90 deg on the spot, so the lidar sits 0.2 m along
    map's +y. Composing the tilt as a yaw — the planar shortcut the odom path
    is allowed — would leave the pitch smeared into the heading here; the 3x3
    composition gives exactly 90.
    """
    turn = math.pi / 2.0
    # map_T_lidar for a robot at the origin yawed 90 deg: the mount rotated
    # onto +y, and the mount's pitch applied after that yaw.
    yaw_q = _quat_for_yaw(turn)
    rot = quat_to_rotation_matrix(*yaw_q) @ quat_to_rotation_matrix(
        *_pitch_quat(MOUNT_PITCH)
    )
    quat = _quat_from_matrix(rot)

    mapping_subs["pointlio/lio_odom"].callback(
        _lio_odom(x=0.0, y=0.2, z=0.4, quat=quat)
    )

    pose = repo.get_pose()
    assert (pose.x, pose.y, pose.z) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert pose.yaw_deg == pytest.approx(90.0)


def test_lio_pose_asks_for_both_halves_of_the_split_tree(mapping_subs, mapping_tf):
    mapping_subs["pointlio/lio_odom"].callback(_lio_odom(x=0.0, y=0.0))

    # The namespace prefix is read off the body frame, never configured — an
    # upstream rename of the LIO frames must not need an edit in the backend.
    assert mapping_tf.lookups == [
        ("map", "robot01/pointlio_odom", rclpy.time.Time()),
        ("robot01/base_link", "robot01/lidar_top", rclpy.time.Time()),
    ]


def test_a_missing_mapping_transform_drops_the_sample_without_raising(
    mapping_subs, repo, mapping_tf
):
    # pgo has not broadcast map->pointlio_odom yet (it publishes nothing until
    # the builder reaches MAPPING). Same rule as the odom path: drop, do not
    # raise into the executor at 20 Hz.
    del mapping_tf.transforms[("map", "robot01/pointlio_odom")]

    mapping_subs["pointlio/lio_odom"].callback(_lio_odom(x=1.0, y=1.0))

    assert repo.get_pose() is None


def test_a_live_odom_feed_silences_the_lio_fallback(mapping_subs, repo, mapping_tf):
    """In AUTO both topics publish; lio_bridge's is the one that counts.

    The stack navigates by the odom feed, so a console drawing the LIO front
    end's raw solution would be showing a second opinion of where the robot is
    — most visibly right after a relocalize, when the two disagree by whatever
    the correction just absorbed.
    """
    mapping_tf.transforms[("map", "robot01/odom")] = _tf(x=5.0)
    mapping_subs["odom"].callback(_odom(x=0.0, y=0.0))
    assert repo.get_pose().x == pytest.approx(5.0)

    mapping_subs["pointlio/lio_odom"].callback(_lio_odom(x=0.2, y=0.0, z=0.4))

    # Unchanged, and the lookup was never even attempted.
    assert repo.get_pose().x == pytest.approx(5.0)
    assert ("map", "robot01/pointlio_odom") not in [
        (target, source) for target, source, _ in mapping_tf.lookups
    ]


# --- joints: motor_states reduced to {joint name: position} -------------------


def _motor_states(joints, timestamp_ns):
    msg = MotorStates()
    msg.timestamp = timestamp_ns
    for name, q in joints:
        motor = MotorState()
        motor.name = name
        motor.q = float(q)
        # Non-zero on purpose: velocity must NOT reach the repo, and a zero
        # would make a leak indistinguishable from a default.
        motor.dq = -1.0
        msg.states.append(motor)
    return msg


def test_motor_states_reduce_to_joint_name_to_position(subs, repo):
    subs["motor_states"].callback(
        _motor_states(
            joints=(("FL_HipX_joint", 0.5), ("FR_Knee_joint", -0.25)),
            timestamp_ns=1_754_000_000_000_000_000,
        )
    )

    sample = repo.get_joints()
    # Exactly {URDF joint name: q} — the vocabulary the frontend uses to look
    # up GLB nodes; dq and temperature stay on the topic.
    assert sample.joints == {"FL_HipX_joint": 0.5, "FR_Knee_joint": -0.25}
    # NANOSECONDS on this topic (seconds in RobotState.motor_status — the
    # aggregator rescales its copy). The 1e-9 here is the unit boundary.
    assert sample.stamp == pytest.approx(1_754_000_000.0)
