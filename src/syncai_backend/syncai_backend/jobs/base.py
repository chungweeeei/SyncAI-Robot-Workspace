"""Abstract base for jobs.

A job is a self-contained unit of background work — for example driving a
navigation goal to completion or polling a long-running operation. Concrete jobs
subclass :class:`BaseJob` and implement :meth:`run`.
"""

from abc import ABC, abstractmethod
from typing import Any


class BaseJob(ABC):
    """Contract for a runnable background job."""

    @abstractmethod
    async def run(self) -> Any:
        """Execute the job to completion and return its result."""
        ...
