"""Abstract base for repositories.

A repository hides the storage backend (in-memory, database, ROS state, ...)
behind a uniform async CRUD interface so the rest of the backend depends on the
abstraction rather than a concrete store. ``T`` is the domain entity type the
concrete repository manages.
"""

from abc import ABC, abstractmethod
from typing import Generic, Optional, TypeVar

T = TypeVar('T')


class BaseRepository(ABC, Generic[T]):
    """Async CRUD contract for a collection of domain entities of type ``T``."""

    @abstractmethod
    async def get(self, entity_id: str) -> Optional[T]:
        """Return the entity with ``entity_id``, or ``None`` if it does not exist."""
        ...

    @abstractmethod
    async def list_all(self) -> list[T]:
        """Return all entities in the collection."""
        ...

    @abstractmethod
    async def create(self, entity: T) -> T:
        """Persist a new entity and return the stored representation."""
        ...

    @abstractmethod
    async def update(self, entity_id: str, entity: T) -> Optional[T]:
        """Replace the entity at ``entity_id``; return it, or ``None`` if absent."""
        ...

    @abstractmethod
    async def delete(self, entity_id: str) -> bool:
        """Remove the entity at ``entity_id``; return ``True`` if one was removed."""
        ...
