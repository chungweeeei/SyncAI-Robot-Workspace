import os
import time
import structlog

from sqlalchemy import (
    Engine,
    create_engine,
    text
)

from sqlalchemy_utils import (
    create_database,
    database_exists
)

MAX_RETRIES=20
RETRY_INTERVAL = 5

def _execute(engine: Engine, clause: str, raise_error: bool = True):

    with engine.connect() as conn:
        try:
            conn.execute(text(clause))
            conn.commit()
        except Exception as e:
            if 'psycopg2.errors.DuplicateColumn' in str(e):
                return

            if raise_error:
                raise e

def connect_to_postgres(logger: structlog.stdlib.BoundLogger) -> Engine:
    pg_user=os.getenv("POSTGRES_USER", "syncrobotic")
    pg_password=os.getenv("POSTGRES_PASSWORD", "syncrobotic")
    pg_host=os.getenv("POSTGRES_HOST", "localhost")
    pg_port=int(os.getenv("POSTGRES_PORT", 5432))

    engine = create_engine(
        f"postgresql://{pg_user}:{pg_password}@{pg_host}:{pg_port}/robot",
        pool_size=5,
        max_overflow=10
    )

    # attempt to connect to the database, retrying if it fails
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            if not database_exists(url=engine.url):
                create_database(url=engine.url)
            return engine
        except Exception as err:
            logger.warning(
                "Connection attempt failed",
                component="Postgres",
                attempt=attempt,
                max_retries=MAX_RETRIES,
                error=str(err),
            )
            if attempt == MAX_RETRIES:
                raise err
            time.sleep(RETRY_INTERVAL)