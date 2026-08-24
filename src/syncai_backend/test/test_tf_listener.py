"""Tests for the backend's shared TF listener wrapper.

TfListener is a thin seam over tf2_ros (a Buffer plus one TransformListener on
the injected node), but its construction-time choices are exactly the ones a
refactor could silently lose, and all of them are observable on a fake node —
tf2_ros's TransformListener only calls node.create_subscription when
spin_thread is False, so no rclpy.init() is needed. Pinned:

* it subscribes to ABSOLUTE ``/tf`` and ``/tf_static``. Topic namespacing is
  the repo-wide landmine, and TF is its one deliberate exception: per
  CLAUDE.md, TF frame names (not topics) carry the robot_id prefix, so the
  topics are fleet-shared and must NOT pick up the node's namespace. That the
  paths start with "/" is therefore load-bearing, not incidental.
* ``/tf_static`` keeps TRANSIENT_LOCAL durability, so the backend (a late
  joiner — it starts seconds after robot_state_publisher) still receives the
  latched static transforms.
* transforms fed through both subscriptions land in the ONE exposed
  ``.buffer`` — the whole reason this wrapper exists is that the point-cloud
  and telemetry subscribers used to keep two copies of the tree.
* ``spin_thread=False``: construction spawns no dedicated listener thread; the
  node's own executor services the callbacks.

The consumers' lookups themselves (lookup_transform_full and friends) are
deliberately NOT wrapped here — see the source docstring — so there is nothing
more of the public surface to pin.
"""

import threading

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("tf2_ros")

import rclpy.qos  # noqa: E402
import rclpy.time  # noqa: E402

from geometry_msgs.msg import TransformStamped  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from tf2_ros.buffer import Buffer  # noqa: E402

from syncai_backend.subscribers.tf import init_tf_listener  # noqa: E402


class FakeNode:
    """Records create_subscription; tf2_ros also destroys subs on __del__."""

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        self.subscriptions.append(
            {
                "msg_type": msg_type,
                "topic": topic,
                "callback": callback,
                "qos": qos_profile,
            }
        )
        return object()

    def destroy_subscription(self, subscription):
        # TransformListener.__del__ unregisters through the node; without this
        # the fake would raise inside GC and pollute unrelated tests' output.
        pass


def make_transform(parent, child, translation=(0.0, 0.0, 0.0), stamp_sec=5):
    tf = TransformStamped()
    tf.header.frame_id = parent
    tf.header.stamp.sec = stamp_sec
    tf.child_frame_id = child
    t = tf.transform.translation
    t.x, t.y, t.z = (float(v) for v in translation)
    tf.transform.rotation.w = 1.0
    return tf


@pytest.fixture
def wired(logger):
    node = FakeNode()
    listener = init_tf_listener(logger=logger, node=node)
    return node, listener


def sub_for(node, topic):
    matches = [s for s in node.subscriptions if s["topic"] == topic]
    assert len(matches) == 1, f"expected exactly one subscription on {topic}"
    return matches[0]


def test_subscribes_to_absolute_tf_and_tf_static(wired):
    node, _ = wired

    assert len(node.subscriptions) == 2
    tf_sub = sub_for(node, "/tf")
    static_sub = sub_for(node, "/tf_static")
    assert tf_sub["msg_type"] is TFMessage
    assert static_sub["msg_type"] is TFMessage


def test_tf_static_keeps_transient_local_durability(wired):
    node, _ = wired

    # Static transforms are published once and latched; a VOLATILE
    # subscription here would leave the late-starting backend without the
    # URDF extrinsics forever.
    static_qos = sub_for(node, "/tf_static")["qos"]
    assert static_qos.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    dynamic_qos = sub_for(node, "/tf")["qos"]
    assert dynamic_qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE


def test_dynamic_transforms_flow_into_the_exposed_buffer(wired):
    node, listener = wired
    assert isinstance(listener.buffer, Buffer)

    # Drive the actual recorded /tf callback with a real TFMessage: this is
    # the whole wiring under test — subscription callback -> the one shared
    # buffer the consumers were injected with.
    sub_for(node, "/tf")["callback"](
        TFMessage(
            transforms=[make_transform("map", "robot01/odom", translation=(1.5, 0.0, 0.0))]
        )
    )

    out = listener.buffer.lookup_transform("map", "robot01/odom", rclpy.time.Time())
    assert out.transform.translation.x == pytest.approx(1.5)


def test_static_transforms_are_valid_at_any_lookup_time(wired):
    node, listener = wired

    sub_for(node, "/tf_static")["callback"](
        TFMessage(
            transforms=[
                make_transform(
                    "robot01/base_link", "robot01/laser", translation=(0.0, 0.0, 0.3)
                )
            ]
        )
    )

    # Statics must answer at times far from their stamp — that is what
    # set_transform_static (vs set_transform) buys, and what the /tf_static
    # callback being the *static* one proves.
    when = rclpy.time.Time(seconds=12345)
    out = listener.buffer.lookup_transform("robot01/base_link", "robot01/laser", when)
    assert out.transform.translation.z == pytest.approx(0.3)


def test_construction_spawns_no_dedicated_spin_thread(logger):
    # spin_thread=False is load-bearing: a private listener thread would
    # contend on the GIL with rclpy, uvicorn and the Temporal worker, and the
    # callbacks are meant to run in the node's default group so the cloud
    # subscriber can block on lookups without starving them.
    before = set(threading.enumerate())
    init_tf_listener(logger=logger, node=FakeNode())
    assert set(threading.enumerate()) == before
