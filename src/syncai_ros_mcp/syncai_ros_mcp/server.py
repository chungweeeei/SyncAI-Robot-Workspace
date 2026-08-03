"""MCP server setup and lifecycle for the SyncAI ROS 2 bridge.

Mirrors syncai_backend's ``interfaces/rest/server.py``: ``init_mcp_server``
builds the server instance, ``start_mcp_server`` runs it in a background daemon
thread so the ROS 2 node can own the main thread for ``rclpy.spin()``.
"""

import threading

from fastmcp import FastMCP
from rclpy.node import Node

from syncai_ros_mcp.tools.maps import register_map_tools
from syncai_ros_mcp.tools.services import register_service_tools
from syncai_ros_mcp.tools.tasks import register_task_tools
from syncai_ros_mcp.tools.topics import register_topic_tools


def init_mcp_server(node: Node) -> FastMCP:
    """Build the FastMCP instance and register all ROS 2 tools."""
    mcp = FastMCP("syncai-ros-mcp")

    register_topic_tools(mcp=mcp, node=node)
    register_service_tools(mcp=mcp, node=node)
    register_task_tools(mcp=mcp)
    register_map_tools(mcp=mcp)

    return mcp


def start_mcp_server(node: Node, host: str = "0.0.0.0", port: int = 8000) -> None:
    """Run the MCP server in a background daemon thread.

    The tool callbacks read the live ROS 2 graph through ``node``, which is
    kept up to date by ``rclpy.spin(node)`` running on the main thread.
    """
    mcp = init_mcp_server(node=node)

    def _run():
        mcp.run(transport="http", host=host, port=port)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
