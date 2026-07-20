import sys
import dotenv
import structlog

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener

from syncai_backend.logger import setup_log_handler
from syncai_backend.database.postgres import connect_to_postgres
from syncai_backend.temporal.worker import start_temporal_worker

from syncai_backend.interfaces.rest.server import start_rest_server

from syncai_backend.repositories.robot.robot import init_robot_repo
from syncai_backend.repositories.map.map import init_map_repo
from syncai_backend.repositories.pointcloud.pointcloud import init_pointcloud_repo

from syncai_backend.gateways.robot.robot import init_robot_gateway
from syncai_backend.gateways.artifact.artifact import init_artifact_gateway
from syncai_backend.gateways.workflow.workflow import init_workflow_gateway

from syncai_backend.subscribers.robot_state_subscriber import (
    init_robot_state_subscriber,
)
from syncai_backend.subscribers.map_subscriber import init_map_subscriber
from syncai_backend.subscribers.pointcloud_subscriber import (
    init_pointcloud_subscriber,
)


dotenv.load_dotenv()
setup_log_handler()
logger = structlog.get_logger()


class SyncAIBackend(Node):
    def __init__(self, logger: structlog.stdlib.BoundLogger):
        super().__init__("syncai_backend_node")

        # The launch file sets the namespace to the robot_id from
        # config/system.ini; it scopes this robot's Temporal task queue so
        # another robot's worker never picks up tasks submitted here.
        robot_id = self.get_namespace().strip("/") or "default_robot"

        try:
            engine = connect_to_postgres(logger=logger, robot_id=robot_id)
        except Exception as e:
            logger.error("Failed to connect to PostgreSQL", error=str(e))
            raise

        robot_repo = init_robot_repo(logger=logger)
        # init_map_repo creates the ORM schema and builds its own session_maker
        # from the engine (per-repo session convention).
        map_repo = init_map_repo(logger=logger, engine=engine)
        pc_repo = init_pointcloud_repo(logger=logger)

        # Shared TF buffer for transforming body_cloud into the map frame; the
        # listener fills it from /tf and /tf_static on this node's executor.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        robot_gw = init_robot_gateway(logger=logger, node=self)
        artifact_gw = init_artifact_gateway(logger=logger)
        workflow_gw = init_workflow_gateway(logger=logger, robot_id=robot_id)

        init_robot_state_subscriber(logger=logger, node=self, robot_repo=robot_repo)
        init_map_subscriber(logger=logger, node=self, map_repo=map_repo)
        init_pointcloud_subscriber(
            logger=logger, node=self, pc_repo=pc_repo, tf_buffer=self._tf_buffer
        )

        start_temporal_worker(
            logger=logger, robot_id=robot_id, robot_gw=robot_gw, artifact_gw=artifact_gw
        )
        start_rest_server(
            logger=logger,
            workflow_gw=workflow_gw,
            robot_repo=robot_repo,
            robot_gw=robot_gw,
            map_repo=map_repo,
            pc_repo=pc_repo,
        )


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
