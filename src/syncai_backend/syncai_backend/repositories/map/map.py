import uuid
import structlog

from contextlib import contextmanager
from typing import Iterator, Optional

from sqlalchemy import Engine, select
from sqlalchemy.orm import Session, sessionmaker

from syncai_backend.database.models import MapPoint


class MapRepo:
    """PostgreSQL-backed CRUD for map vertices.

    This repo used to also cache the live ``map`` topic's OccupancyGrid, for two
    REST endpoints that reported the loaded map. Those are served per map name
    off the filesystem now, so the cache, its only writer (``map_subscriber.py``)
    and this repo's dependency on ROS message types all went with them.

    The engine is required. It used to be ``Optional``, threaded through from
    the older wiring, and every method opened with a "was there an engine?"
    guard — a branch for a state the only caller (``main.py``) cannot produce,
    since it raises when the Postgres connection fails long before getting here.
    Demanding the engine in the constructor makes that state unrepresentable
    instead of merely checked.
    """

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
    def _session(self, op: str) -> Iterator[Session]:
        """A session for one repo operation, naming ``op`` if it fails.

        Log-and-reraise, deliberately: the SQLAlchemy error carries the
        statement that failed, so translating it into a repo-level exception
        would trade a diagnosable traceback for a constant string. The REST
        layer already turns an unhandled error into a 500, and the domain
        exceptions in ``syncai_backend.exceptions`` are the supported route
        for anything that wants a different status.

        No explicit rollback: ``Session.__exit__`` closes the session, and
        closing rolls back whatever was left uncommitted.
        """
        with self.session_maker() as session:
            try:
                yield session
            except Exception:
                self.logger.error(
                    f"[MapRepo][{op}] database operation failed", exc_info=True
                )
                raise

    def create_vertices(self, vertices: list[dict]) -> list[MapPoint]:
        """Batch-insert vertices in a single transaction.

        Each dict carries the column values (name/type/map_name/x/y/theta).
        All rows are committed together, so a failure inserts none of them.
        """
        with self._session("create_vertices") as session:
            rows = [MapPoint(**fields) for fields in vertices]
            session.add_all(rows)
            session.commit()
            for row in rows:
                session.refresh(row)
            return rows

    def list_vertices(
        self, map_name: Optional[str] = None, type: Optional[str] = None
    ) -> list[MapPoint]:
        """Vertices matching both filters, oldest first.

        The filters are optional and AND together; omitting both lists every
        vertex on the robot, and callers treat an empty result as normal (a map
        with no vertices yet), so this must never raise on zero rows.
        """
        with self._session("list_vertices") as session:
            stmt = select(MapPoint)
            if map_name is not None:
                stmt = stmt.where(MapPoint.map_name == map_name)
            if type is not None:
                stmt = stmt.where(MapPoint.type == type)
            # id is a random UUID, so order by creation time for a stable,
            # meaningful listing order.
            stmt = stmt.order_by(MapPoint.created_at)
            # scalars(), not query(): Session.query() takes entities, not an
            # already-built Select, and returns Rows rather than MapPoints.
            return list(session.scalars(stmt).all())

    def get_vertex(self, vertex_id: uuid.UUID) -> Optional[MapPoint]:
        with self._session("get_vertex") as session:
            return session.get(MapPoint, vertex_id)

    def update_vertex(self, vertex_id: uuid.UUID, **fields) -> Optional[MapPoint]:
        # Only these columns may be updated through the API.
        allowed = {"name", "type", "map_name", "x", "y", "theta"}
        changes = {k: v for k, v in fields.items() if k in allowed and v is not None}

        with self._session("update_vertex") as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return None

            for key, value in changes.items():
                setattr(vertex, key, value)

            session.commit()
            session.refresh(vertex)
            return vertex

    def delete_vertex(self, vertex_id: uuid.UUID) -> bool:
        with self._session("delete_vertex") as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return False

            session.delete(vertex)
            session.commit()
            return True


def init_map_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Engine,
) -> MapRepo:
    MapPoint.metadata.create_all(engine)
    return MapRepo(logger=logger, engine=engine)
