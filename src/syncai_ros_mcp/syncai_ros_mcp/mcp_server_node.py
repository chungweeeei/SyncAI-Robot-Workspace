import rclpy
from rclpy.node import Node

from syncai_ros_mcp.server import start_mcp_server


class SyncAIROSMcpServer(Node):

    def __init__(self):
        super().__init__("syncai_mcp_server_node")

        # Start the MCP server in a background daemon thread so this node can
        # own the main thread for rclpy.spin(); see start_mcp_server.
        start_mcp_server(node=self)


def main():
    rclpy.init(args=None)

    try:
        node = SyncAIROSMcpServer()
        rclpy.spin(node)
    except Exception:
        rclpy.logging.get_logger("syncai_mcp_server_node").error(
            "Failed to execute SyncAI ROS MCP server node"
        )
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
