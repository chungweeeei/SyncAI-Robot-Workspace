import sys
import dotenv
import structlog

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from syncai_backend.logger import setup_log_handler
from syncai_backend.database.postgres import connect_to_postgres
from syncai_backend.temporal.worker import start_temporal_worker

from syncai_backend.interfaces.rest.server import start_rest_server

from syncai_backend.repositories.robot.robot import init_robot_repo
from syncai_backend.repositories.map.map import init_map_repo
from syncai_backend.repositories.task.saved_task import init_saved_task_repo
from syncai_backend.repositories.map.catalog import init_map_catalog_repo
from syncai_backend.repositories.pointcloud.pointcloud import init_pointcloud_repo
from syncai_backend.repositories.telemetry.telemetry import init_telemetry_repo

from syncai_backend.gateways.robot.robot import init_robot_gateway
from syncai_backend.gateways.map.map import init_map_gateway
from syncai_backend.gateways.artifact.artifact import init_artifact_gateway
from syncai_backend.gateways.workflow.workflow import init_workflow_gateway

from syncai_backend.subscribers.robot_state_subscriber import (
    init_robot_state_subscriber,
)
from syncai_backend.subscribers.pointcloud_subscriber import (
    init_pointcloud_subscriber,
)
from syncai_backend.subscribers.telemetry_subscriber import (
    init_telemetry_subscriber,
)
from syncai_backend.subscribers.path_subscriber import (
    init_path_subscriber,
)
from syncai_backend.subscribers.tf import init_tf_listener


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
        # The operator's library of re-dispatchable step lists. Its own repo
        # rather than a method on MapRepo: that one is the vertex table, this is a
        # different table, and the only thing they share is the engine.
        saved_task_repo = init_saved_task_repo(logger=logger, engine=engine)
        # The maps on disk, as opposed to the one that is loaded. Reads the
        # filesystem only; no engine, no ROS.
        map_catalog_repo = init_map_catalog_repo(logger=logger)
        # Single-slot cache for the live body_cloud, drained by the WS stream.
        # The static map cloud used to have a second one; it is read from the
        # saved .pcd on request now, so there is nothing to cache between calls.
        pointcloud_repo = init_pointcloud_repo(logger=logger)
        # Single-slot pose/joints cache feeding the internal telemetry WS
        # (the high-rate channel the 3D viewer uses instead of the frozen,
        # whole-second-resolution GET /api/v1/robot/state contract).
        telemetry_repo = init_telemetry_repo(logger=logger)

        robot_gw = init_robot_gateway(logger=logger, node=self)
        # LoadMap client, so an edited gridmap can be pushed into the running
        # map_server. Its own gateway rather than a method on RobotGateway: that
        # one is the driver / sys_manager / nav surface (motion keys, wifi,
        # initialpose, NavigateToPose), and the map router has no business
        # holding a handle that can command the robot to move.
        map_gw = init_map_gateway(logger=logger, node=self)
        artifact_gw = init_artifact_gateway(logger=logger)
        workflow_gw = init_workflow_gateway(logger=logger, robot_id=robot_id)

        # One /tf + /tf_static subscription for the whole process, shared by the
        # two subscribers that need transforms. Held on self because this is the
        # object that owns it; see subscribers/tf.py for why they no longer
        # build one each.
        self._tf_listener = init_tf_listener(logger=logger, node=self)

        init_robot_state_subscriber(logger=logger, node=self, robot_repo=robot_repo)
        init_pointcloud_subscriber(
            logger=logger,
            node=self,
            pointcloud_repo=pointcloud_repo,
            tf_buffer=self._tf_listener.buffer,
        )
        init_telemetry_subscriber(
            logger=logger,
            node=self,
            telemetry_repo=telemetry_repo,
            tf_buffer=self._tf_listener.buffer,
        )
        # The planner's route, onto the same telemetry socket. No tf_buffer: the
        # plan is published in the map frame already.
        init_path_subscriber(logger=logger, node=self, telemetry_repo=telemetry_repo)

        worker_handle = start_temporal_worker(
            logger=logger, robot_id=robot_id, robot_gw=robot_gw, artifact_gw=artifact_gw
        )
        start_rest_server(
            logger=logger,
            workflow_gw=workflow_gw,
            robot_repo=robot_repo,
            robot_gw=robot_gw,
            map_repo=map_repo,
            map_catalog_repo=map_catalog_repo,
            map_gw=map_gw,
            pointcloud_repo=pointcloud_repo,
            telemetry_repo=telemetry_repo,
            saved_task_repo=saved_task_repo,
            worker_handle=worker_handle,
        )


def main():
    rclpy.init(args=None)

    try:
        backend_node = SyncAIBackend(logger=logger)
        # MultiThreadedExecutor lets the point-cloud callback group run on a
        # separate thread from the robot_state/map/TF callbacks, so a busy scan
        # frame can't starve them.
        executor = MultiThreadedExecutor()
        executor.add_node(backend_node)
        executor.spin()
    except Exception:
        logger.error("Failed to execute SyncAIBackend node", exc_info=True)
    finally:
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
