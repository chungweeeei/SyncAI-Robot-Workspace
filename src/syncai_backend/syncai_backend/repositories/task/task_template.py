import uuid
import structlog

from collections.abc import Generator
from contextlib import contextmanager
from typing import Any, Optional

from sqlalchemy import Engine, or_, select
from sqlalchemy.orm import Session, sessionmaker

from syncai_backend.database.models import TaskTemplate


class TaskTemplateRepo:
    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Engine,
    ):
        self.logger = logger

        # Register database session factory (per-repo session convention).
        self.session_maker = sessionmaker(
            autocommit=False,
            autoflush=False,
            expire_on_commit=False,
            bind=engine,
        )

    @contextmanager
    def _session(self, op: str) -> Generator[Session, None, None]:
        """A session for one repo operation, naming ``op`` if it fails.

        The ``Generator`` annotation (not ``Iterator``) is what ``@contextmanager``
        wants -- see the note on ``MapRepo._session``.

        Log-and-reraise, for the reason ``MapRepo._session`` records at length:
        the SQLAlchemy error carries the statement that failed, and translating
        it into a repo-level exception would trade a diagnosable traceback for a
        constant string.
        """
        with self.session_maker() as session:
            try:
                yield session
            except Exception:
                self.logger.error(
                    f"[TaskTemplateRepo][{op}] database operation failed", exc_info=True
                )
                raise

    def create_task_template(
        self,
        *,
        name: str,
        description: str,
        map_name: Optional[str],
        steps: list[dict[str, Any]],
    ) -> TaskTemplate:
        with self._session("create_task_template") as session:
            row = TaskTemplate(
                name=name,
                description=description,
                map_name=map_name,
                steps=steps,
            )
            session.add(row)
            session.commit()
            session.refresh(row)
            return row

    def list_task_templates(
        self,
        map_name: Optional[str] = None,
        include_map_independent: bool = True,
    ) -> list[TaskTemplate]:
        """Task templates, by name then creation time.

        ``map_name`` cannot be overloaded as "no filter" the way ``MapRepo``
        overloads its own, because ``map_name IS NULL`` is a real query here: it
        is what a posture-only, run-anywhere template looks like. So a supplied
        ``map_name`` means "this map's templates *and* the map-independent ones"
        -- the console's actual question -- and ``include_map_independent=False``
        narrows it to strictly that map. ``map_name=None`` still means every row.

        Ordered by name rather than ``updated_at DESC``: unlike the vertex list
        (where the DB order is meaningless and the client sorts), this one is
        rendered as-is, and putting the most recently saved row first would
        reshuffle the list under the operator's cursor after every save. Names
        are not unique, so ``created_at`` breaks ties.
        """
        with self._session("list_task_templates") as session:
            stmt = select(TaskTemplate)
            if map_name is not None:
                stmt = stmt.where(
                    or_(
                        TaskTemplate.map_name == map_name,
                        TaskTemplate.map_name.is_(None),
                    )
                    if include_map_independent
                    else TaskTemplate.map_name == map_name
                )
            stmt = stmt.order_by(TaskTemplate.name, TaskTemplate.created_at)
            return list(session.scalars(stmt).all())

    def get_task_template(self, task_id: uuid.UUID) -> Optional[TaskTemplate]:
        with self._session("get_task_template") as session:
            return session.get(TaskTemplate, task_id)

    def update_task_template(self, task_id: uuid.UUID, **fields) -> Optional[TaskTemplate]:
        # Only these columns may be updated through the API.
        allowed = {"name", "description", "map_name", "steps"}
        # Deliberately NOT filtering `v is not None`, unlike MapRepo.update_vertex:
        # `map_name=None` is a meaningful value here -- clearing it is how a
        # template becomes map-independent -- and dropping Nones would make that
        # transition inexpressible. The router's `model_dump(exclude_unset=True)`
        # is what guarantees an *omitted* field never arrives here as a None in
        # the first place, so the guard MapRepo needs is not needed and would be
        # a bug.
        changes = {k: v for k, v in fields.items() if k in allowed}

        with self._session("update_task_template") as session:
            row = session.get(TaskTemplate, task_id)
            if row is None:
                return None

            for key, value in changes.items():
                # Whole-list assignment for `steps`, never an in-place mutation:
                # JSON columns are not mutation-tracked, so an append would be
                # silently dropped. See the note in database/models.py.
                setattr(row, key, value)

            session.commit()
            session.refresh(row)
            return row

    def delete_task_template(self, task_id: uuid.UUID) -> bool:
        with self._session("delete_task_template") as session:
            row = session.get(TaskTemplate, task_id)
            if row is None:
                return False

            session.delete(row)
            session.commit()
            return True


def init_task_template_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Engine,
) -> TaskTemplateRepo:
    # This looks redundant and is not quite. `Base.metadata` is shared, so
    # `init_map_repo`'s create_all already emits `task_templates` too -- but
    # keeping the call here makes each repo factory self-sufficient (its contract
    # is "the factory creates its own schema") and makes construction order in
    # main.py irrelevant. The cost is one reflection round trip at startup,
    # because checkfirst=True is the default.
    TaskTemplate.metadata.create_all(engine)
    return TaskTemplateRepo(logger=logger, engine=engine)
