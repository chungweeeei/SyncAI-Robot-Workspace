"""Tests for the live body_cloud subscriber (msg in -> repo slot out).

The subscriber is wired to a fake node so the recorded callback can be invoked
directly with hand-built sensor_msgs/PointCloud2 messages — no rclpy.init(), no
executor. What is pinned is the pipeline's observable contract, not its mocks:

* registration: the RELATIVE topic name (namespacing comes from the node, per
  the repo-wide robot_id convention) and the sensor-data QoS;
* a cloud already in ``map`` is packed into the single-slot repo untouched,
  without consulting TF at all;
* a body-frame cloud is transformed via ``lookup_transform_full`` split at the
  cloud frame's TF parent — latest time for map->odom, the cloud stamp for
  odom->body — because both single-time variants are known-wrong (the long
  comment in the source records the measured failure modes). The lookup
  arguments are load-bearing, so the happy path asserts them alongside the
  transformed floats;
* TF miss / no-parent-yet frames are DROPPED (slot unchanged), and the stream
  recovers on the next resolvable frame;
* empty clouds and NaN rows never reach the repo, frames beyond the cap are
  thinned, and a new frame replaces the slot (seq advances — single-slot
  semantics, the WS pump's dedup cursor depends on it).

pack_xyz_f32 itself is covered by test_pointcloud.py; here it only defines the
expected byte layout of the slot.
"""

import pytest

pytest.importorskip("numpy")
pytest.importorskip("rclpy")
pytest.importorskip("tf2_ros")
pytest.importorskip("sensor_msgs_py")

import numpy as np  # noqa: E402
import rclpy.qos  # noqa: E402
import yaml  # noqa: E402

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from tf2_ros import ExtrapolationException  # noqa: E402

from syncai_backend.repositories.pointcloud.pointcloud import (  # noqa: E402
    init_pointcloud_repo,
)
from syncai_backend.subscribers.pointcloud_subscriber import (  # noqa: E402
    init_pointcloud_subscriber,
)

BODY_FRAME = "robot01/pointlio_body"
ODOM_FRAME = "robot01/pointlio_odom"


class FakeNode:
    """Records create_subscription calls; the source only ever calls that."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        self.subscriptions.append(
            {
                "msg_type": msg_type,
                "topic": topic,
                "callback": callback,
                "qos": qos_profile,
                "callback_group": kwargs.get("callback_group"),
            }
        )
        return object()


class FakeTfBuffer:
    """Just the two Buffer methods the subscriber touches, with recording.

    ``parents`` drives all_frames_as_yaml (the tree dump _resolve_fixed_frame
    parses for the split point); ``transform``/``error`` drive
    lookup_transform_full. Both are plain attributes so a test can flip the
    buffer between "TF missing" and "TF back" mid-stream.
    """

    def __init__(self, parents=None, transform=None, error=None):
        self.parents = dict(parents or {})
        self.transform = transform
        self.error = error
        self.lookups = []
        self.tree_reads = 0

    def all_frames_as_yaml(self):
        self.tree_reads += 1
        return yaml.safe_dump(
            {frame: {"parent": parent} for frame, parent in self.parents.items()}
        )

    def lookup_transform_full(self, **kwargs):
        self.lookups.append(kwargs)
        if self.error is not None:
            raise self.error
        return self.transform


def make_cloud(points, frame_id="map", stamp_sec=0, is_dense=False):
    """A real PointCloud2 with the layout the subscriber reads.

    x/y/z as little-endian FLOAT32 at offsets 0/4/8, point_step 12,
    is_bigendian False — the layout pointlio publishes (minus its extra
    fields, which read_points_numpy ignores by offset anyway). is_dense
    defaults to False because read_points only honours skip_nans then.
    """
    arr = np.asarray(points, dtype="<f4").reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = stamp_sec
    msg.height = 1
    msg.width = arr.shape[0]
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate("xyz")
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * arr.shape[0]
    msg.data = arr.tobytes()
    msg.is_dense = is_dense
    return msg


def make_transform(translation=(0.0, 0.0, 0.0), quat_xyzw=(0.0, 0.0, 0.0, 1.0)):
    tf = TransformStamped()
    t = tf.transform.translation
    t.x, t.y, t.z = (float(v) for v in translation)
    q = tf.transform.rotation
    q.x, q.y, q.z, q.w = (float(v) for v in quat_xyzw)
    return tf


def slot_points(repo, after_seq=0):
    frame = repo.get_latest(after_seq=after_seq)
    assert frame is not None
    pts = np.frombuffer(frame.data, dtype="<f4").reshape(-1, 3)
    # num_points and the byte payload must agree — the WS pump prefixes the
    # count the client uses to size its BufferGeometry.
    assert frame.num_points == pts.shape[0]
    return frame, pts


@pytest.fixture
def repo(logger):
    return init_pointcloud_repo(logger=logger)


@pytest.fixture
def wire(logger, repo):
    """(fake node, fake buffer, recorded callback) with the subscriber wired up."""
    def _wire(tf_buffer):
        node = FakeNode()
        init_pointcloud_subscriber(
            logger=logger, node=node, pointcloud_repo=repo, tf_buffer=tf_buffer
        )
        assert len(node.subscriptions) == 1
        return node, tf_buffer, node.subscriptions[0]["callback"]

    return _wire


def test_subscribes_to_the_relative_body_cloud_topic_best_effort(wire):
    node, _, _ = wire(FakeTfBuffer())
    sub = node.subscriptions[0]

    # Relative on purpose: the node's namespace (robot_id) scopes it. An
    # absolute /<robot_id>/... here has been a real bug before.
    assert sub["topic"] == "pointlio/body_cloud"
    assert sub["msg_type"] is PointCloud2

    qos = sub["qos"]
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE
    assert qos.depth == 5

    # Own group so a busy scan frame cannot starve the /tf callbacks it
    # blocks on (the lookup timeout in _cloud_cb depends on this).
    assert isinstance(sub["callback_group"], MutuallyExclusiveCallbackGroup)


def test_map_frame_cloud_is_packed_without_any_tf_lookup(wire, repo):
    _, buffer, callback = wire(FakeTfBuffer())

    callback(make_cloud([(1.0, 2.0, 3.0), (-4.5, 0.0, 9.25)], frame_id="map"))

    frame, pts = slot_points(repo)
    assert frame.seq == 1
    assert pts.tolist() == [[1.0, 2.0, 3.0], [-4.5, 0.0, 9.25]]
    # Already in the target frame: transforming again would be a no-op bought
    # with a lookup that can fail, so TF must not be touched at all.
    assert buffer.lookups == []
    assert buffer.tree_reads == 0


def test_body_frame_cloud_is_transformed_using_the_split_time_lookup(wire, repo):
    # +90 deg about z, then translate: (1,0,0) -> (10,-1,0.5), (0,1,0) -> (9,-2,0.5).
    tf = make_transform(
        translation=(10.0, -2.0, 0.5), quat_xyzw=(0.0, 0.0, 0.7071068, 0.7071068)
    )
    _, buffer, callback = wire(
        FakeTfBuffer(parents={BODY_FRAME: ODOM_FRAME}, transform=tf)
    )

    callback(
        make_cloud([(1.0, 0.0, 0.0), (0.0, 1.0, 0.0)], frame_id=BODY_FRAME, stamp_sec=7)
    )

    _, pts = slot_points(repo)
    assert pts[0] == pytest.approx([10.0, -1.0, 0.5], abs=1e-5)
    assert pts[1] == pytest.approx([9.0, -2.0, 0.5], abs=1e-5)

    # The lookup arguments carry the whole point of lookup_transform_full:
    # map<-odom at latest (the lagging ICP correction), odom<-body at the
    # cloud's own stamp, split at the frame's TF parent read from the tree.
    (lookup,) = buffer.lookups
    assert lookup["target_frame"] == "map"
    assert lookup["source_frame"] == BODY_FRAME
    assert lookup["fixed_frame"] == ODOM_FRAME
    assert lookup["target_time"].nanoseconds == 0
    assert lookup["source_time"].nanoseconds == 7 * 10**9
    # The timeout covers the intra-process cloud-vs-/tf delivery race; without
    # it ~20% of frames were dropped, so its presence is behavior, not tuning.
    assert lookup["timeout"].nanoseconds == pytest.approx(0.05 * 10**9)


def test_tf_miss_drops_the_frame_and_a_later_frame_recovers(wire, repo):
    buffer = FakeTfBuffer(
        parents={BODY_FRAME: ODOM_FRAME},
        transform=make_transform(translation=(1.0, 0.0, 0.0)),
        error=ExtrapolationException("not relocalized yet"),
    )
    _, _, callback = wire(buffer)

    callback(make_cloud([(5.0, 5.0, 5.0)], frame_id=BODY_FRAME))
    # Dropped, not published untransformed: a body-frame cloud in the map slot
    # would draw the scan at the origin regardless of the robot's pose.
    assert repo.get_latest() is None

    buffer.error = None
    callback(make_cloud([(0.0, 0.0, 0.0)], frame_id=BODY_FRAME))
    frame, pts = slot_points(repo)
    assert frame.seq == 1
    assert pts.tolist() == [[1.0, 0.0, 0.0]]


def test_cloud_frame_without_a_tf_parent_is_dropped_before_any_lookup(wire, repo):
    # Empty tree: pointlio not broadcasting yet. _resolve_fixed_frame raises
    # its own LookupException, which must ride the same drop path as a miss.
    _, buffer, callback = wire(FakeTfBuffer(parents={}))

    callback(make_cloud([(1.0, 2.0, 3.0)], frame_id=BODY_FRAME))

    assert repo.get_latest() is None
    assert buffer.lookups == []


def test_fixed_frame_is_resolved_once_and_cached_per_source_frame(wire, repo):
    buffer = FakeTfBuffer(
        parents={BODY_FRAME: ODOM_FRAME}, transform=make_transform()
    )
    _, _, callback = wire(buffer)

    callback(make_cloud([(1.0, 1.0, 1.0)], frame_id=BODY_FRAME))
    # Wipe the tree dump: if the second frame still transforms, the split
    # point came from the cache, i.e. the tree is only consulted once per
    # source frame (its shape does not change at runtime, only its values).
    buffer.parents = {}
    callback(make_cloud([(2.0, 2.0, 2.0)], frame_id=BODY_FRAME))

    frame, pts = slot_points(repo)
    assert frame.seq == 2
    assert pts.tolist() == [[2.0, 2.0, 2.0]]
    assert buffer.tree_reads == 1


def test_empty_cloud_leaves_the_slot_untouched(wire, repo):
    _, buffer, callback = wire(FakeTfBuffer())

    callback(make_cloud(np.zeros((0, 3)), frame_id="map"))

    assert repo.get_latest() is None
    assert buffer.lookups == []


def test_nan_points_are_dropped_before_packing(wire, repo):
    _, _, callback = wire(FakeTfBuffer())

    callback(
        make_cloud(
            [(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0)], frame_id="map", is_dense=False
        )
    )

    _, pts = slot_points(repo)
    assert pts.tolist() == [[1.0, 2.0, 3.0]]


def test_frames_beyond_the_cap_are_strided_down(wire, repo):
    _, _, callback = wire(FakeTfBuffer())
    n = 30001  # one over the subscriber's _max_points

    xs = np.arange(n, dtype="<f4")
    callback(make_cloud(np.column_stack([xs, xs, xs]), frame_id="map"))

    frame, pts = slot_points(repo)
    assert 0 < frame.num_points <= 30000
    # cap_points is a stride, not a truncation — the thinned cloud must still
    # span the whole scan, not just its first 30000 points.
    assert pts[-1, 0] == pytest.approx(n - 1, abs=2.0)


def test_a_new_frame_replaces_the_slot(wire, repo):
    _, _, callback = wire(FakeTfBuffer())

    callback(make_cloud([(1.0, 1.0, 1.0)], frame_id="map"))
    callback(make_cloud([(2.0, 2.0, 2.0), (3.0, 3.0, 3.0)], frame_id="map"))

    frame, pts = slot_points(repo)
    assert frame.seq == 2  # seq advanced: the WS pump's dedup cursor moves on
    assert pts.tolist() == [[2.0, 2.0, 2.0], [3.0, 3.0, 3.0]]
    # Single-slot: the first frame is gone, not queued behind the second.
    assert repo.get_latest(after_seq=frame.seq) is None
