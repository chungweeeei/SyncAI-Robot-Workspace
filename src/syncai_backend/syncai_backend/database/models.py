"""SQLAlchemy ORM models persisted in the per-robot ``<robot_id>_db`` PostgreSQL database."""

import uuid
from datetime import datetime, timezone
from typing import Any, Optional

from sqlalchemy import JSON, DateTime, Float, String, Uuid
from sqlalchemy.dialects.postgresql import JSONB
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

    __tablename__ = "map_vertices"

    id: Mapped[uuid.UUID] = mapped_column(
        Uuid, primary_key=True, default=uuid.uuid4
    )
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    # Free-form classification, e.g. "waypoint" / "task_point".
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


class SavedTask(Base):
    """An operator-authored step list, kept so it can be re-dispatched.

    Temporal is not the library. Namespace ``default`` retains closed workflows
    for one day with no archival configured, so a dispatched task's steps are
    gone by tomorrow; the ``/tasks`` composer's output has to live somewhere that
    survives a page reload, and this is it.

    ``steps`` is a JSON array rather than a child table, for four reasons in the
    order they matter here:

    1. **This package has no migrations.** The schema is whatever ``create_all``
       produced on first boot. A typed child table would freeze the *step
       vocabulary* into DDL nothing here can subsequently alter -- ``ArtifactParams``
       alone is a discriminated command union of five fields, and the next step
       type would need an ``ALTER TABLE`` on a robot with no tool to run one. A
       JSON array grows an optional key and the old rows keep validating.
    2. **The step schema is already enforced above the storage layer.**
       ``StepRequest`` + ``validate_step_params`` is the single authority on which
       step types carry which params, and it runs at the request boundary. Typed
       columns would be a second encoding of that rule, in a place that cannot
       express "MOVE requires params, STANDUP forbids them" at all.
    3. **A JSON array *is* the order.** A child table needs a position column,
       and because editing a step list means reordering and inserting, every
       update degenerates to delete-all + insert-all anyway. The child table
       would buy typed columns for the identical whole-list rewrite, plus a
       position column that can go wrong.
    4. **SQLite has to work** -- ``test/conftest.py`` runs an in-memory engine,
       and ``sqlalchemy.JSON`` is supported on both dialects.

    Forward-compatibility rule that the JSON choice exists to serve, and which
    must be honoured: only ever *add* optional keys to a stored step object.
    Never rename, retype, or repurpose one. A blob written by a newer backend is
    read by an older one on a rollback.

    Two traps:

    - **JSON columns are not mutation-tracked.** ``row.steps.append(...)`` inside
      a session is silently not flushed. ``SavedTaskRepo`` only ever assigns a
      fresh list, which is why ``MutableList.as_mutable`` is not used -- but the
      first person to try an in-place edit will lose it, hence this note.
    - **``Base.metadata`` is shared.** The moment this class exists,
      ``init_map_repo``'s ``MapPoint.metadata.create_all(engine)`` -- the same
      statement spelled through a different class -- also creates ``saved_tasks``.
      That is harmless, but it means adding a model to this file changes what an
      unrelated repo factory does.
    """

    __tablename__ = "saved_tasks"

    id: Mapped[uuid.UUID] = mapped_column(Uuid, primary_key=True, default=uuid.uuid4)
    # A label, not an identity: two saved tasks may share a name, exactly as two
    # map vertices may (map_vertices has no unique constraint either), and the id
    # is what addresses one. A unique constraint was considered and rejected --
    # there is no migration path to add or drop one later, and it could not cover
    # the map-independent rows anyway, since both PostgreSQL and SQLite treat
    # NULLs in a unique index as distinct.
    name: Mapped[str] = mapped_column(String(255), nullable=False)
    description: Mapped[str] = mapped_column(String(1000), nullable=False, default="")
    # The map whose frame this task's MOVE coordinates are in, or NULL when the
    # task has no MOVE step and is therefore runnable anywhere. Indexed for the
    # "what can I run on the map the robot is actually on" listing.
    map_name: Mapped[Optional[str]] = mapped_column(
        String(255), nullable=True, index=True
    )
    # Ordered saved steps; the element shape is ``_StoredStep`` in
    # interfaces/rest/routers/saved_task.py. JSONB on PostgreSQL -- indexable and
    # comparable with ``=``, which plain ``json`` is not -- and the variant has to
    # be chosen now, because there is no ALTER path later. Plain JSON on SQLite,
    # which is what the test fixture runs.
    steps: Mapped[list[dict[str, Any]]] = mapped_column(
        JSON().with_variant(JSONB(), "postgresql"), nullable=False
    )
    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, nullable=False
    )
    updated_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True), default=_utcnow, onupdate=_utcnow, nullable=False
    )
