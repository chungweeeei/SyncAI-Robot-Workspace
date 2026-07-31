import uuid
import structlog

from typing import Optional

from sqlalchemy import Engine, select
from sqlalchemy.orm import sessionmaker

from syncai_backend.database.models import MapPoint


class MapRepo:
    """PostgreSQL-backed CRUD for map vertices.

    Every method blocks on psycopg2, so the REST handlers that call them are
    declared with plain ``def`` for FastAPI's worker thread pool.

    An SQLAlchemy ``Engine`` is passed in and the repo builds its own
    ``session_maker`` from it (same convention as CoreManager's TaskRepo).
    ``expire_on_commit=False`` keeps returned ORM instances readable after the
    session closes, so the router can serialise them.

    This repo used to also cache the live ``map`` topic's OccupancyGrid, for two
    REST endpoints that reported the loaded map. Those are served per map name
    off the filesystem now, so the cache, its only writer (``map_subscriber.py``)
    and this repo's dependency on ROS message types all went with them. The
    engine stays ``Optional`` because it is threaded through the same wiring as
    before; the methods raise if it is absent.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Optional[Engine] = None,
    ):
        self.logger = logger

        # Register database session factory (None when no engine was given).
        self.session_maker: Optional[sessionmaker] = (
            sessionmaker(
                autocommit=False,
                autoflush=False,
                expire_on_commit=False,
                bind=engine,
            )
            if engine is not None
            else None
        )

    def _sessions(self) -> sessionmaker:
        if self.session_maker is None:
            raise RuntimeError("MapRepo was created without a database engine.")
        return self.session_maker

    def create_vertices(self, vertices: list[dict]) -> list[MapPoint]:
        """Batch-insert vertices in a single transaction.

        Each dict carries the column values (name/type/map_name/x/y/theta).
        All rows are committed together, so a failure inserts none of them.
        """
        with self._sessions()() as session:
            rows = [MapPoint(**fields) for fields in vertices]
            session.add_all(rows)
            session.commit()
            for row in rows:
                session.refresh(row)
            return rows

    def list_vertices(
        self, map_name: Optional[str] = None, type: Optional[str] = None
    ) -> list[MapPoint]:
        with self._sessions()() as session:
            stmt = select(MapPoint)
            if map_name is not None:
                stmt = stmt.where(MapPoint.map_name == map_name)
            if type is not None:
                stmt = stmt.where(MapPoint.type == type)
            # id is a random UUID, so order by creation time for a stable,
            # meaningful listing order.
            stmt = stmt.order_by(MapPoint.created_at)
            return list(session.scalars(stmt).all())

    def get_vertex(self, vertex_id: uuid.UUID) -> Optional[MapPoint]:
        with self._sessions()() as session:
            return session.get(MapPoint, vertex_id)

    def update_vertex(self, vertex_id: uuid.UUID, **fields) -> Optional[MapPoint]:
        # Only these columns may be updated through the API.
        allowed = {"name", "type", "map_name", "x", "y", "theta"}
        changes = {k: v for k, v in fields.items() if k in allowed and v is not None}

        with self._sessions()() as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return None

            for key, value in changes.items():
                setattr(vertex, key, value)

            session.commit()
            session.refresh(vertex)
            return vertex

    def delete_vertex(self, vertex_id: uuid.UUID) -> bool:
        with self._sessions()() as session:
            vertex = session.get(MapPoint, vertex_id)
            if vertex is None:
                return False

            session.delete(vertex)
            session.commit()
            return True


def init_map_repo(
    logger: structlog.stdlib.BoundLogger,
    engine: Optional[Engine] = None,
) -> MapRepo:
    if engine is not None:
        MapPoint.metadata.create_all(engine)
    return MapRepo(logger=logger, engine=engine)
