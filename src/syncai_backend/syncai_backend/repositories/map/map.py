import uuid
import structlog

from collections.abc import Generator
from contextlib import contextmanager
from typing import Optional, TypedDict

from sqlalchemy import Engine, select
from sqlalchemy.orm import Session, sessionmaker

from syncai_backend.database.models import MapPoint


class VertexFields(TypedDict):
    name: str
    type: str
    x: float
    y: float
    theta: float


class MapRepo:
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
        with self.session_maker() as session:
            try:
                yield session
            except Exception:
                self.logger.error(f"[MapRepo][{op}] database operation failed", exc_info=True)
                raise

    def create_vertices(self, map: str, vertices: list[VertexFields]) -> list[MapPoint]:
        """Batch-insert vertices in a single transaction.

        Each dict carries the column values (name/type/x/y/theta).
        All rows are committed together, so a failure inserts none of them.
        """
        with self._session("create_vertices") as session:
            rows = [MapPoint(map=map, **fields) for fields in vertices]
            session.add_all(rows)
            session.commit()
            return rows

    def list_vertices(
        self, map: Optional[str] = None, type: Optional[str] = None
    ) -> list[MapPoint]:
        """Vertices matching both filters, oldest first.

        The filters are optional and AND together; omitting both lists every
        vertex on the robot, and callers treat an empty result as normal (a map
        with no vertices yet), so this must never raise on zero rows.
        """
        with self._session("list_vertices") as session:
            stmt = select(MapPoint)
            if map is not None:
                stmt = stmt.where(MapPoint.map == map)
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
        allowed = {"name", "type", "x", "y", "theta"}
        changes = {k: v for k, v in fields.items() if k in allowed and v is not None}

        with self._session("update_vertex") as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return None

            for key, value in changes.items():
                setattr(vertex, key, value)

            session.commit()
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
