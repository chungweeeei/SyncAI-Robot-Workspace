"""Abstract base for HTTP routers.

A router groups related REST endpoints behind a URL prefix and builds a
configured :class:`fastapi.APIRouter` that the application factory mounts onto
the FastAPI app. Concrete routers subclass :class:`BaseRouter`, declare their
``prefix`` / ``tags``, and register their routes in :meth:`build`.
"""

from abc import ABC, abstractmethod

from fastapi import APIRouter


class BaseRouter(ABC):
    """Contract for a mountable group of REST endpoints.

    Attributes:
        prefix: URL prefix all routes in this router live under (e.g. ``/robot``).
        tags: OpenAPI tags applied to every route, used for docs grouping.
    """

    prefix: str = ''
    tags: list[str] = []

    @abstractmethod
    def build(self) -> APIRouter:
        """Create and return the configured ``APIRouter`` with routes attached."""
        ...
