"""Tests for the pgo map-cloud subscriber (merged keyframe cloud -> repo slot).

Same harness as test_pointcloud_subscriber.py: a fake node records the
subscription, and the recorded callback is fed real hand-built
sensor_msgs/PointCloud2 messages. Pinned here is what makes this subscriber
deliberately NOT a copy of the live-cloud one:

* no TF anywhere — the constructor does not even take a buffer, because pgo
  publishes the merge already in the map frame;
* depth-1 QoS on the RELATIVE ``pgo/map_cloud`` topic (a queued older merge is
  never worth delivering — each message replaces the map wholesale);
* each merge REPLACES the single slot (a loop closure moves the whole map, so
  deltas are impossible) with num_points/bytes agreeing;
* empty clouds are ignored and NaN rows are dropped before packing.

pack_xyz_f32 / cap_points have their own tests in test_pointcloud.py; this file
only asserts the slot's resulting floats.
"""

import pytest

pytest.importorskip("numpy")
pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs_py")

import numpy as np  # noqa: E402
import rclpy.qos  # noqa: E402

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402

from syncai_backend.repositories.pointcloud.pointcloud import (  # noqa: E402
    init_pointcloud_repo,
)
from syncai_backend.subscribers.map_cloud_subscriber import (  # noqa: E402
    init_map_cloud_subscriber,
)


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


def make_cloud(points, frame_id="map", is_dense=False):
    """A real PointCloud2 in the layout the subscriber reads.

    x/y/z little-endian FLOAT32 at offsets 0/4/8, point_step 12 — what pgo's
    pcl::toROSMsg produces for a plain XYZ cloud. is_dense stays False by
    default because read_points only honours skip_nans then.
    """
    arr = np.asarray(points, dtype="<f4").reshape(-1, 3)
    msg = PointCloud2()
    msg.header.frame_id = frame_id
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


def slot_points(repo):
    frame = repo.get_latest()
    assert frame is not None
    pts = np.frombuffer(frame.data, dtype="<f4").reshape(-1, 3)
    # num_points and the payload must agree — the WS pump prefixes the count
    # the frontend uses to size its buffers.
    assert frame.num_points == pts.shape[0]
    return frame, pts


@pytest.fixture
def repo(logger):
    return init_pointcloud_repo(logger=logger)


@pytest.fixture
def wire(logger, repo):
    """(fake node, recorded callback) with the subscriber wired up."""
    node = FakeNode()
    init_map_cloud_subscriber(logger=logger, node=node, map_cloud_repo=repo)
    assert len(node.subscriptions) == 1
    return node, node.subscriptions[0]["callback"]


def test_subscribes_to_the_relative_pgo_topic_with_a_depth_one_queue(wire):
    node, _ = wire
    sub = node.subscriptions[0]

    # Relative on purpose: the node's namespace (robot_id) scopes it, and this
    # subscription existing is what un-gates pgo's subscriber-gated publisher.
    assert sub["topic"] == "pgo/map_cloud"
    assert sub["msg_type"] is PointCloud2

    qos = sub["qos"]
    # Depth 1 where the live cloud uses 5: each merge replaces the map
    # wholesale, so delivering a queued older merge is pure waste.
    assert qos.depth == 1
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE

    # Own group: parsing a multi-MB merge must not starve the 10 Hz
    # body_cloud / state / TF callbacks on the shared executor.
    assert isinstance(sub["callback_group"], MutuallyExclusiveCallbackGroup)


def test_a_merge_lands_in_the_slot_as_packed_f32_xyz(wire, repo):
    _, callback = wire

    callback(make_cloud([(1.0, 2.0, 3.0), (-4.5, 0.0, 9.25)]))

    frame, pts = slot_points(repo)
    assert frame.seq == 1
    assert pts.tolist() == [[1.0, 2.0, 3.0], [-4.5, 0.0, 9.25]]
    assert frame.data == np.asarray(
        [(1.0, 2.0, 3.0), (-4.5, 0.0, 9.25)], dtype="<f4"
    ).tobytes()


def test_the_cloud_is_taken_as_is_without_a_map_frame_check(wire, repo):
    # No TF path exists in this subscriber (the constructor takes no buffer),
    # so the header frame is trusted, not verified: pgo places every point
    # with the keyframes' corrected global poses at merge time.
    _, callback = wire

    callback(make_cloud([(1.0, 0.0, 0.0)], frame_id="robot01/anything"))

    _, pts = slot_points(repo)
    assert pts.tolist() == [[1.0, 0.0, 0.0]]


def test_each_merge_replaces_the_slot_wholesale(wire, repo):
    _, callback = wire

    callback(make_cloud([(1.0, 1.0, 1.0)]))
    callback(make_cloud([(2.0, 2.0, 2.0), (3.0, 3.0, 3.0)]))

    frame, pts = slot_points(repo)
    assert frame.seq == 2  # seq advanced: the WS pump's dedup cursor moves on
    assert pts.tolist() == [[2.0, 2.0, 2.0], [3.0, 3.0, 3.0]]
    # Single-slot: after a loop closure the OLD map must be unreachable, not
    # queued behind the new one.
    assert repo.get_latest(after_seq=frame.seq) is None


def test_empty_cloud_leaves_the_slot_untouched(wire, repo):
    _, callback = wire

    callback(make_cloud(np.zeros((0, 3))))

    assert repo.get_latest() is None


def test_nan_points_are_dropped_before_packing(wire, repo):
    _, callback = wire

    callback(make_cloud([(1.0, 2.0, 3.0), (float("nan"), 0.0, 0.0)], is_dense=False))

    _, pts = slot_points(repo)
    assert pts.tolist() == [[1.0, 2.0, 3.0]]
