"""SQLAlchemy ORM models persisted in the ``robot_db`` PostgreSQL database."""

from datetime import datetime, timezone

from sqlalchemy import DateTime, Float, Integer, String
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


class Base(DeclarativeBase):
    """Declarative base shared by every ORM model."""


class MapPoint(Base):
    """A named point placed on a map, stored in world (map-frame) coordinates.

    Coordinates mirror the navigation goal convention (``x``/``y`` in metres,
    ``theta`` in degrees, expressed in the ``map`` frame), so a stored point can
    be handed straight to a MOVE step without conversion.
    """

    __tablename__ = "map_point"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    # Free-form classification, e.g. "waypoint" / "task_point" / "patrol".
    type: Mapped[str] = mapped_column(String(64), nullable=False)
    # Which map this point belongs to; indexed for per-map listing.
    map_name: Mapped[str] = mapped_column(String(255), nullable=False, index=True)
    x: Mapped[float] = mapped_column(Float, nullable=False)
    y: Mapped[float] = mapped_column(Float, nullable=False)
    theta: Mapped[float] = mapped_column(Float, nullable=False)
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, onupdate=_utcnow, nullable=False
    )
