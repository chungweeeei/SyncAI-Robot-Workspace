"""Entry point for the robot backend.

Runs a ROS 2 node and a FastAPI (uvicorn) HTTP server in one process:

  - rclpy spins in a background thread on a MultiThreadedExecutor, so ROS
    callbacks (subscriptions, action feedback, timers) keep flowing while
    HTTP requests are served.
  - uvicorn runs on the main thread and blocks until shutdown.

Host/port are ROS parameters so they can be set from launch or the CLI:
  ros2 run syncai_backend backend --ros-args -p port:=9000
"""

import threading

import rclpy
import uvicorn
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from syncai_backend.app import create_app


class BackendNode(Node):
    """ROS 2 node that owns the robot's ROS interfaces for the REST API.

    Declare publishers / subscribers / action clients here; the FastAPI route
    handlers reach them through `request.app.state.ros_node`.
    """

    def __init__(self) -> None:
        super().__init__('syncai_backend')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        # TODO: declare ROS interfaces (publishers / action clients / ...) here.


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BackendNode()

    # Spin ROS in the background so HTTP serving does not block ROS callbacks.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info(
        f'syncai_backend REST API listening on http://{node.host}:{node.port} '
        f'(docs at /docs)'
    )

    app = create_app(node)
    config = uvicorn.Config(app, host=node.host, port=node.port, log_level='info')
    server = uvicorn.Server(config)

    try:
        server.run()  # blocks until Ctrl-C / shutdown
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
