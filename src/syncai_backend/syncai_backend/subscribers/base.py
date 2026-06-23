"""Abstract base for ROS 2 subscribers.

A subscriber binds a single ROS 2 topic to a callback that updates backend state
(caches, repositories, broadcast queues). Concrete subscribers declare their
``topic`` and message type and implement :meth:`on_message`; :meth:`subscribe`
wires the subscription onto a live node.
"""

from abc import ABC, abstractmethod
from typing import Any

from rclpy.node import Node


class BaseSubscriber(ABC):
    """Contract for a single-topic ROS 2 subscription handler.

    Attributes:
        topic: Name of the ROS 2 topic to subscribe to.
        msg_type: ROS 2 message class carried on the topic.
        qos: Quality-of-Service depth/profile for the subscription.
    """

    topic: str = ''
    msg_type: type = object
    qos: int = 10

    @abstractmethod
    def on_message(self, msg: Any) -> None:
        """Handle a single inbound message from :attr:`topic`."""
        ...

    @abstractmethod
    def subscribe(self, node: Node) -> None:
        """Create the subscription on ``node``, routing messages to ``on_message``."""
        ...
