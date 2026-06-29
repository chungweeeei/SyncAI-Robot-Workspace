import sys
import rclpy
import dotenv
import structlog
from fastapi import FastAPI

from logger import setup_log_handler
from syncai_backend.database.postgres import connect_to_postgres

dotenv.load_dotenv()
setup_log_handler()
logger = structlog.get_logger()

if __name__ == "__main__":
    rclpy.init(args=None)

    try:
        db_engine = connect_to_postgres(logger=logger)
    except Exception as e:
        logger.error("Failed to connect to PostgreSQL", error=str(e))
        rclpy.shutdown()
        sys.exit(1)
