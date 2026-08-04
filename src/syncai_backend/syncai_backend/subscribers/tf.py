import structlog

from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TfListener:
    """The backend's single /tf + /tf_static subscription.

    Built once in ``SyncAIBackend.__init__`` and injected into every subscriber
    that needs a transform, the same way the repos and gateways are.

    The point-cloud and telemetry subscribers used to build a Buffer and
    TransformListener each, on the argument that sharing one would couple their
    lifecycles. It does not: both are constructed unconditionally in the same
    ``__init__``, on the same node, spun by the same executor, and neither can
    be started or stopped without the other. There was no lifecycle to couple —
    only a second copy of the whole TF tree, a second /tf callback doing
    identical work, and the possibility of the two subscribers transiently
    disagreeing about the tree they were both reading.

    Consumers get the raw ``buffer``, NOT a wrapped ``lookup()``. Their lookups
    are not interchangeable — the telemetry subscriber wants map->odom at the
    latest time, the cloud subscriber needs ``lookup_transform_full`` split at
    the odom frame with a different time for each half, plus
    ``all_frames_as_yaml()`` to find that split point — and each keeps its own
    edge-triggered "TF went away" logging, with its own message and its own
    frame pair. A shared wrapper would have to carry per-consumer state to say
    anything useful, which is exactly what the consumers already do.
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger, node: Node):
        self.logger = logger

        self.buffer = Buffer()
        # spin_thread=False: the listener's subscriptions are serviced by the
        # node's own executor rather than a private one, so it never spawns an
        # extra GIL-contending thread next to rclpy, uvicorn and the Temporal
        # worker. They land in the node's default callback group, which is what
        # lets PointCloudSubscriber block on a lookup timeout inside its own
        # MutuallyExclusive group without starving the transforms it is waiting
        # for.
        #
        # Retained on self (and in turn by main.py) even though rclpy already
        # keeps it alive transitively: create_subscription appends to the node's
        # subscription list, and each Subscription holds the bound callback,
        # i.e. this object. The explicit reference is not lifetime insurance,
        # it is what makes the ownership visible at the one place that owns it.
        self._listener = TransformListener(self.buffer, node, spin_thread=False)


def init_tf_listener(logger: structlog.stdlib.BoundLogger, node: Node) -> TfListener:
    return TfListener(logger=logger, node=node)
