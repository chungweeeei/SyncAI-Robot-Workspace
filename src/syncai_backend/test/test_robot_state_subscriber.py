"""Tests for RobotStateSubscriber: the ``robot_state`` topic -> RobotRepo hand-off.

Two decisions are pinned, both of which have been made and unmade once already:

* The message is stored WHOLE and UNFILTERED. The REST router downstream is the
  whitelist (test_robot_router pins that); filtering here too would mean two
  places to widen every time a field is deliberately exposed. Identity is
  asserted, not equality — equality would quietly pass a copy that drops fields
  the router has not learnt about yet.

* Unlocalized samples (``localization_valid=false``, zeroed pose) are stored.
  The subscriber used to drop them, which 404'd the whole state endpoint —
  mode, battery and gait state included — for the entire mapping run, whose TF
  chain never reaches base_link. A regression here blinds the mapping console
  again; the pose-honesty half of the old trade rides on the flag now.

The subscription is checked against a fake node rather than a live graph: the
topic must stay the RELATIVE name ``robot_state`` (an absolute topic name in a
backend subscriber is a bug that has already been fixed once — CLAUDE.md), and
the QoS must stay best-effort so a slow backend sheds aggregator samples
instead of queueing them.
"""

from types import SimpleNamespace

import pytest

pytest.importorskip("syncai_common")
pytest.importorskip("rclpy")

import rclpy.qos  # noqa: E402

from syncai_common.msg import RobotState as RobotStateMsg  # noqa: E402

from syncai_backend.repositories.robot.robot import init_robot_repo  # noqa: E402
from syncai_backend.subscribers.robot_state_subscriber import (  # noqa: E402
    init_robot_state_subscriber,
)


class _FakeNode:
    """Records create_subscription calls instead of touching the DDS graph.

    ``create_subscription`` is the only Node surface the subscriber uses, and
    driving the recorded callback by hand is the whole harness — no
    ``rclpy.init()``, no executor.
    """

    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos_profile):
        sub = SimpleNamespace(
            msg_type=msg_type, topic=topic, callback=callback, qos_profile=qos_profile
        )
        self.subscriptions.append(sub)
        return sub


@pytest.fixture
def robot_repo(logger):
    return init_robot_repo(logger=logger)


@pytest.fixture
def node():
    return _FakeNode()


@pytest.fixture
def subscription(logger, node, robot_repo):
    """The one subscription init_robot_state_subscriber registers."""
    init_robot_state_subscriber(logger=logger, node=node, robot_repo=robot_repo)
    (sub,) = node.subscriptions
    return sub


def test_subscribes_to_the_relative_robot_state_topic(subscription):
    # Relative, so it inherits the robot_id namespace from the backend node.
    # One DDS domain hosts several robots; an absolute name here would read
    # every robot's aggregate (and has, once).
    assert subscription.topic == "robot_state"
    assert subscription.msg_type is RobotStateMsg


def test_qos_is_best_effort_keep_last(subscription):
    qos = subscription.qos_profile

    # Best-effort volatile: the aggregator republishes at 10 Hz, so a lost
    # sample is 100 ms away and queueing stale ones helps nobody.
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE
    assert qos.history == rclpy.qos.HistoryPolicy.KEEP_LAST
    assert qos.depth == 3


def test_the_whole_message_lands_in_the_repo_unfiltered(
    subscription, robot_repo, make_robot_state
):
    msg = make_robot_state()

    subscription.callback(msg)

    # Identity, not equality: the subscriber hands the message through, and the
    # router's whitelist is the only projection. A copy that trimmed fields
    # would still compare equal today and silently lose tomorrow's field.
    assert robot_repo.get_robot_state() is msg


def test_a_second_sample_replaces_the_first(subscription, robot_repo, make_robot_state):
    subscription.callback(make_robot_state(timestamp=1754000000))
    later = make_robot_state(timestamp=1754000001, battery_percentage=50.0)

    subscription.callback(later)

    # RobotRepo is a single slot — GET /state serves only the newest aggregate.
    assert robot_repo.get_robot_state() is later


def test_an_unlocalized_sample_is_stored_not_dropped(
    subscription, robot_repo, make_robot_state
):
    """The mapping-mode shape: zeroed pose, localization_valid=false.

    Dropping these is the past bug this file exists to hold shut — it made
    mode, battery and the gait state unreadable exactly when they matter.
    """
    msg = make_robot_state(
        mode=1,  # RobotMode.MANUAL: mapping is where these samples dominate
        localization_valid=False,
        position=(0.0, 0.0, 0.0, 0.0),
    )

    subscription.callback(msg)

    stored = robot_repo.get_robot_state()
    assert stored is msg
    assert stored.localization_valid is False
    # The zeroed pose arrives labelled rather than not arriving.
    assert stored.localization_status.position.x == 0.0
