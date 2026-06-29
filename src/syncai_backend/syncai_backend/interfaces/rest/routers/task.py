# import structlog
# from typing import List
# from fastapi import APIRouter

# from gateways.workflow.schema import Task, TaskRequest, TaskResponse, TaskStatus
# from gateways.workflow.workflow import WorkflowGateway


# def init_task_router(
#     logger: structlog.stdlib.BoundLogger,
# ) -> APIRouter:
#     task_router = APIRouter(prefix="", tags=["Task"])

#     @task_router.post("/api/v1/tasks")
#     async def trigger_task():
#         return

#     @task_router.get("/api/v1/tasks")
#     async def list_tasks():
#         return []

#     @task_router.get("/api/v1/tasks/{id}")
#     async def get_task(id: str):
#         retrun

#     @task_router.delete("/api/v1/tasks/{id}", response_model=TaskResponse)
#     async def cancel_task(id: str):
#         await workflow_gw.cancel_task(task_id=id)
#         return TaskResponse(
#             id=id,
#             status=TaskStatus.CANCELLED,
#             message="Task cancel requested",
#         )

#     return task_router
