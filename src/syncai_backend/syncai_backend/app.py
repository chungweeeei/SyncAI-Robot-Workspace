"""FastAPI application factory for the syncai_backend REST API.

The app is created with a reference to the running ROS 2 node, so route
handlers can publish to topics, call services, or send action goals through it.
Add your routes / routers here.
"""

from fastapi import FastAPI
from rclpy.node import Node


def create_app(node: Node) -> FastAPI:
    """Build the FastAPI app, wiring it to the live ROS 2 node.

    Keep `node` out of module-global state and pass it in here so the API layer
    stays testable and there is a single owner of the ROS interfaces.
    """
    app = FastAPI(
        title='SyncAI Robot Backend',
        description='RESTful API for the SyncAI robot, bridged to ROS 2.',
        version='0.0.0',
    )

    # Stash the node on app.state so route modules can reach it via the request:
    #   request.app.state.ros_node
    app.state.ros_node = node

    @app.get('/health', tags=['system'])
    def health() -> dict:
        """Liveness probe — confirms the backend node is up and spinning."""
        return {'status': 'ok', 'node': node.get_name()}

    # --- Add your routers below ---------------------------------------------
    # e.g.
    #   from syncai_backend.routers import navigation
    #   app.include_router(navigation.router, prefix='/navigate', tags=['nav'])
    # ------------------------------------------------------------------------

    return app
