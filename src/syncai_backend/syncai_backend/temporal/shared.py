import os

# Temporal server frontend gRPC endpoint. Defaults to the local docker-compose
# service (`temporal:7233` inside the compose network / `127.0.0.1:7233` on host).
TEMPORAL_SERVER_URL = os.getenv("TEMPORAL_ADDRESS", "127.0.0.1:7233")

# Task queue the worker polls and workflows are dispatched to.
TASK_QUEUE = os.getenv("TEMPORAL_TASK_QUEUE", "syncai-task-queue")


def get_workflow_id(task_id: str) -> str:
    return f"task-{task_id}"
