"""Abstract base for gateways.

A gateway encapsulates communication with an outbound system the backend drives
— typically a ROS 2 action server, service, or topic, but possibly an external
HTTP API. It exposes a domain-oriented interface so callers stay decoupled from
the transport. Concrete gateways subclass :class:`BaseGateway`.
"""

from abc import ABC, abstractmethod


class BaseGateway(ABC):
    """Contract for an adapter to an outbound system."""

    @abstractmethod
    async def connect(self) -> None:
        """Establish/initialise the underlying client (action client, channel, ...)."""
        ...

    @abstractmethod
    async def close(self) -> None:
        """Release any resources held by the gateway."""
        ...
