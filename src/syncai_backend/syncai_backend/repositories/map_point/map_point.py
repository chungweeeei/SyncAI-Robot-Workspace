import structlog

from typing import Optional

from sqlalchemy import select
from sqlalchemy.orm import sessionmaker

from syncai_backend.database.models import MapPoint


class MapPointRepo:
    """PostgreSQL-backed CRUD for map points.

    Methods are synchronous because the psycopg2 driver blocks; the REST
    handlers that call them are declared with plain ``def`` so FastAPI runs
    them in its worker thread pool rather than on the event loop (same rationale
    as ``routers/network.py``).
    """

    def __init__(
        self, logger: structlog.stdlib.BoundLogger, session_factory: sessionmaker
    ):
        self._logger = logger
        self._session_factory = session_factory

    def create(
        self, name: str, type: str, map_name: str, x: float, y: float, theta: float
    ) -> MapPoint:
        with self._session_factory() as session:
            point = MapPoint(
                name=name, type=type, map_name=map_name, x=x, y=y, theta=theta
            )
            session.add(point)
            session.commit()
            session.refresh(point)
            return point

    def list_all(
        self, map_name: Optional[str] = None, type: Optional[str] = None
    ) -> list[MapPoint]:
        with self._session_factory() as session:
            stmt = select(MapPoint)
            if map_name is not None:
                stmt = stmt.where(MapPoint.map_name == map_name)
            if type is not None:
                stmt = stmt.where(MapPoint.type == type)
            stmt = stmt.order_by(MapPoint.id)
            return list(session.scalars(stmt).all())

    def get(self, point_id: int) -> Optional[MapPoint]:
        with self._session_factory() as session:
            return session.get(MapPoint, point_id)

    def update(self, point_id: int, **fields) -> Optional[MapPoint]:
        # Only these columns may be updated through the API.
        allowed = {"name", "type", "map_name", "x", "y", "theta"}
        changes = {k: v for k, v in fields.items() if k in allowed and v is not None}

        with self._session_factory() as session:
            point = session.get(MapPoint, point_id)
            if point is None:
                return None

            for key, value in changes.items():
                setattr(point, key, value)

            session.commit()
            session.refresh(point)
            return point

    def delete(self, point_id: int) -> bool:
        with self._session_factory() as session:
            point = session.get(MapPoint, point_id)
            if point is None:
                return False

            session.delete(point)
            session.commit()
            return True


def init_map_point_repo(
    logger: structlog.stdlib.BoundLogger, session_factory: sessionmaker
) -> MapPointRepo:
    return MapPointRepo(logger=logger, session_factory=session_factory)
