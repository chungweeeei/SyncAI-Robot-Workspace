"""Task tools for the ROS 2 MCP server.

Unlike the other tool modules (which introspect the live ROS 2 graph), these
tools are thin HTTP clients for ``syncai_backend``'s task API
(``interfaces/rest/routers/task.py``):

* ``POST   /api/v1/tasks``       -> create/queue a task
* ``GET    /api/v1/tasks/{id}``  -> read per-step task state
* ``DELETE /api/v1/tasks/{id}``  -> request task cancellation

The backend base URL defaults to ``http://localhost:3000`` (the port
``syncai_backend`` binds in ``interfaces/rest/server.py``) and can be overridden
with the ``SYNCAI_BACKEND_BASE_URL`` environment variable.
"""

import time

from fastmcp import FastMCP
from mcp.types import ToolAnnotations

from syncai_ros_mcp.tools import _backend


def register_task_tools(mcp: FastMCP) -> None:
    """Register all task-related tools."""

    @mcp.tool(
        description=(
            "Create and queue a task on the SyncAI backend (POST /api/v1/tasks).\n"
            "A task is an ordered list of steps. Each step is "
            "{'id': str, 'type': 'MOVE', 'params': {...}} where "
            "params depend on the type:\n"
            "  MOVE     -> {'x': float, 'y': float, 'theta': float}  "
            "# theta in degrees, -180 < theta <= 180\n"
            "Example:\n"
            "create_task(task_id='robot01-task-001', steps=[{'id': 'step1', 'type': 'MOVE', "
            "'params': {'x': 1.0, 'y': 2.0, 'theta': 90.0}}])"
        ),
        annotations=ToolAnnotations(title="Create Task"),
    )
    def create_task(task_id: str, steps: list = None, timestamp: int = None) -> dict:
        """
        Create and queue a task on the SyncAI backend.

        Args:
            task_id (str): Unique task identifier (e.g. 'robot01-task-001').
            steps (list): Ordered steps, each a dict with 'id', 'type', and
                'params'. Must contain at least one step.
            timestamp (int): Unix epoch seconds. Defaults to the current time
                when omitted.

        Returns:
            dict: The backend TaskResponse ({'id', 'status', 'message'}) on
                success, or {'error': ...} on validation/transport failure.
        """
        task_id = (task_id or "").strip()
        if not task_id:
            return {"error": "task_id cannot be empty"}

        if not steps:
            return {"error": "steps cannot be empty; provide at least one step"}

        if not isinstance(steps, list):
            return {"error": "steps must be a list of step objects"}

        payload = {
            "id": task_id,
            "timestamp": int(timestamp) if timestamp is not None else int(time.time()),
            "steps": steps,
        }

        return _backend.request("POST", "/api/v1/tasks", json=payload)

    @mcp.tool(
        description=(
            "Get the current state (overall status + per-step status) of a task "
            "(GET /api/v1/tasks/{id}).\n"
            "Example:\nget_task_state('robot01-task-001')"
        ),
        annotations=ToolAnnotations(title="Get Task State", readOnlyHint=True),
    )
    def get_task_state(task_id: str) -> dict:
        """
        Get the current state of a task.

        Args:
            task_id (str): The task identifier.

        Returns:
            dict: The backend TaskStateResponse ({'id', 'status', 'steps'}) on
                success, or {'error': ...} on failure.
        """
        task_id = (task_id or "").strip()
        if not task_id:
            return {"error": "task_id cannot be empty"}

        return _backend.request("GET", f"/api/v1/tasks/{task_id}")

    @mcp.tool(
        description=(
            "Request cancellation of a task (DELETE /api/v1/tasks/{id}).\n"
            "Example:\ncancel_task('robot01-task-001')"
        ),
        annotations=ToolAnnotations(title="Cancel Task", destructiveHint=True),
    )
    def cancel_task(task_id: str) -> dict:
        """
        Request cancellation of a task.

        Args:
            task_id (str): The task identifier.

        Returns:
            dict: The backend TaskResponse ({'id', 'status', 'message'}) on
                success, or {'error': ...} on failure.
        """
        task_id = (task_id or "").strip()
        if not task_id:
            return {"error": "task_id cannot be empty"}

        return _backend.request("DELETE", f"/api/v1/tasks/{task_id}")
