import uuid
import threading
import structlog

from typing import Optional

from nav_msgs.msg import OccupancyGrid
from sqlalchemy import Engine, select
from sqlalchemy.orm import sessionmaker

from syncai_backend.database.models import MapPoint


class MapRepo:
    """The map's in-memory OccupancyGrid cache plus PostgreSQL-backed vertex CRUD.

    The cache methods (``update_map``/``get_map``) only need the logger. The
    vertex methods block on psycopg2, so the REST handlers that call them are
    declared with plain ``def`` for FastAPI's worker thread pool.

    An SQLAlchemy ``Engine`` is passed in and the repo builds its own
    ``session_maker`` from it (same convention as CoreManager's TaskRepo).
    ``expire_on_commit=False`` keeps returned ORM instances readable after the
    session closes, so the router can serialise them. The engine is optional:
    cache-only callers (e.g. the map subscriber, cache tests) omit it, and the
    vertex methods raise if it is absent.
    """

    def __init__(
        self,
        logger: structlog.stdlib.BoundLogger,
        engine: Optional[Engine] = None,
    ):
        self.logger = logger

        # In-Process memory cache for the latest map (OccupancyGrid).
        self._map_lock = threading.Lock()
        self._map: Optional[OccupancyGrid] = None

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

    # --- Map cache ----------------------------------------------------------

    def update_map(self, grid: OccupancyGrid):
        with self._map_lock:
            self._map = grid

    def get_map(self) -> Optional[OccupancyGrid]:
        with self._map_lock:
            return self._map

    # --- Vertex CRUD --------------------------------------------------------

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
