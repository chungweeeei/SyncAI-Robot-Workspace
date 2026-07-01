import sys
import dotenv
import structlog

import rclpy
from rclpy.node import Node

from syncai_backend.logger import setup_log_handler
from syncai_backend.database.postgres import connect_to_postgres
from syncai_backend.temporal.worker import start_temporal_worker

from syncai_backend.interfaces.rest.server import start_rest_server

from syncai_backend.gateways.robot.robot import init_robot_gateway
from syncai_backend.gateways.workflow.workflow import init_workflow_gateway

dotenv.load_dotenv()
setup_log_handler()
logger = structlog.get_logger()


class SyncAIBackend(Node):
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        super().__init__("syncai_backend_node")

        try:
            _ = connect_to_postgres(logger=logger)
        except Exception as e:
            logger.error("Failed to connect to PostgreSQL", error=str(e))
            raise

        robot_gw = init_robot_gateway(logger=logger, node=self)
        workflow_gw = init_workflow_gateway(logger=logger)

        start_temporal_worker(logger=logger, robot_gw=robot_gw)
        start_rest_server(logger=logger, workflow_gw=workflow_gw)


def main():
    rclpy.init(args=None)

    try:
        backend_node = SyncAIBackend(logger=logger)
        rclpy.spin(backend_node)
    except Exception:
        logger.error("Failed to execute SyncAIBackend node", exc_info=True)
    finally:
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
