import threading
import uvicorn
import structlog
from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from syncai_backend.exceptions import (
    BadRequestError,
    ConflictError,
    InternalServerError,
    NotFoundError,
    UnauthorizedError,
)

from syncai_backend.interfaces.rest.routers.task import init_task_router
from syncai_backend.interfaces.rest.routers.schedule import init_schedule_router
from syncai_backend.interfaces.rest.routers.saved_task import init_saved_task_router
from syncai_backend.interfaces.rest.routers.robot import init_robot_router
from syncai_backend.interfaces.rest.routers.network import init_network_router
from syncai_backend.interfaces.rest.routers.map import init_map_router
from syncai_backend.interfaces.rest.routers.pointcloud import init_pointcloud_router
from syncai_backend.interfaces.rest.routers.telemetry import init_telemetry_router

from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.repositories.map.map import MapRepo
from syncai_backend.repositories.map.catalog import MapCatalogRepo
from syncai_backend.repositories.pointcloud.pointcloud import PointCloudRepo
from syncai_backend.repositories.telemetry.telemetry import TelemetryRepo
from syncai_backend.repositories.task.saved_task import SavedTaskRepo

from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.gateways.robot.robot import RobotGateway
from syncai_backend.gateways.map.map import MapGateway

from syncai_backend.temporal.worker import TemporalWorkerHandle


def register_exception_handlers(app: FastAPI) -> None:
    """Map domain exceptions to HTTP responses so routers don't have to."""

    def _json(status_code: int, detail: str) -> JSONResponse:
        return JSONResponse(status_code=status_code, content={"detail": detail})

    @app.exception_handler(NotFoundError)
    async def _not_found(_: Request, exc: NotFoundError) -> JSONResponse:
        return _json(status.HTTP_404_NOT_FOUND, str(exc))

    @app.exception_handler(BadRequestError)
    async def _bad_request(_: Request, exc: BadRequestError) -> JSONResponse:
        return _json(status.HTTP_400_BAD_REQUEST, str(exc))

    @app.exception_handler(ConflictError)
    async def _conflict(_: Request, exc: ConflictError) -> JSONResponse:
        return _json(status.HTTP_409_CONFLICT, str(exc))

    @app.exception_handler(UnauthorizedError)
    async def _unauthorized(_: Request, exc: UnauthorizedError) -> JSONResponse:
        return _json(status.HTTP_401_UNAUTHORIZED, str(exc))

    @app.exception_handler(InternalServerError)
    async def _internal_server(_: Request, exc: InternalServerError) -> JSONResponse:
        return _json(status.HTTP_502_BAD_GATEWAY, str(exc))


def init_rest_server(
    logger: structlog.stdlib.BoundLogger,
    workflow_gw: WorkflowGateway,
    robot_repo: RobotRepo,
    robot_gw: RobotGateway,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
    pointcloud_repo: PointCloudRepo,
    telemetry_repo: TelemetryRepo,
    saved_task_repo: SavedTaskRepo,
    worker_handle: TemporalWorkerHandle,
) -> FastAPI:

    description = """
    This is the backend for the SyncAI Robotic System. It provides APIs for controlling and monitoring the robot, as well as managing data and workflows.
    """

    app = FastAPI(
        title="SyncAI Robot backend Server", description=description, version="1.0.0"
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
        allow_headers=["Content-Type", "Content-Length", "Authorization"],
    )

    register_exception_handlers(app=app)

    @app.get("/health")
    async def health() -> dict:
        """Liveness probe, plus the task server's actual state.

        The Temporal worker runs in a daemon thread whose death used to be
        invisible: POST /tasks kept answering 200 (the gateway lazy-connects)
        while nothing would ever execute the queued work. The handle makes
        that state readable here.

        Always HTTP 200 — a non-200 wired into a container healthcheck would
        restart-loop the whole backend whenever Temporal is down for long,
        and the rest of the process (telemetry, maps, manual control) is
        still healthy. Degradation lives in the body. The field says
        "task_server", not "temporal_worker": REST consumers get the
        operator-facing vocabulary, not the implementation (the vertex /
        MapPoint precedent).
        """
        worker_status, worker_error = worker_handle.snapshot()
        return {
            "status": (
                "ok"
                if worker_status == TemporalWorkerHandle.STATUS_RUNNING
                else "degraded"
            ),
            "task_server": worker_status,
            "task_server_error": worker_error,
        }

    app.include_router(init_task_router(logger=logger, workflow_gw=workflow_gw))
    app.include_router(init_schedule_router(logger=logger, workflow_gw=workflow_gw))
    app.include_router(
        init_robot_router(logger=logger, robot_repo=robot_repo, robot_gw=robot_gw)
    )
    app.include_router(init_network_router(logger=logger, robot_gw=robot_gw))
    # Serves /api/v1/maps/...: the catalogue on disk plus the vertex table. The
    # gateway is here for the save path only — writing a gridmap has to tell the
    # running map_server to re-read it.
    app.include_router(
        init_map_router(
            logger=logger,
            map_repo=map_repo,
            map_catalog_repo=map_catalog_repo,
            map_gw=map_gw,
        )
    )
    # Serves /api/v1/saved_tasks: the operator's library of re-dispatchable step
    # lists. It needs map_repo to resolve a saved MOVE step's vertex reference
    # against that vertex's *current* pose, map_catalog_repo to answer "is this
    # task's map the one the robot is on", and workflow_gw for the one route that
    # freezes a saved task into a Temporal schedule.
    #
    # Included after the task router, though nothing depends on the order: the
    # /api/v1/saved_tasks prefix cannot collide with /api/v1/tasks/{id}, which is
    # precisely why that prefix was chosen over /api/v1/tasks/saved.
    app.include_router(
        init_saved_task_router(
            logger=logger,
            saved_task_repo=saved_task_repo,
            map_repo=map_repo,
            map_catalog_repo=map_catalog_repo,
            workflow_gw=workflow_gw,
        )
    )
    app.include_router(
        init_pointcloud_router(logger=logger, pointcloud_repo=pointcloud_repo)
    )
    app.include_router(
        init_telemetry_router(logger=logger, telemetry_repo=telemetry_repo)
    )

    return app


def start_rest_server(
    logger: structlog.stdlib.BoundLogger,
    workflow_gw: WorkflowGateway,
    robot_repo: RobotRepo,
    robot_gw: RobotGateway,
    map_repo: MapRepo,
    map_catalog_repo: MapCatalogRepo,
    map_gw: MapGateway,
    pointcloud_repo: PointCloudRepo,
    telemetry_repo: TelemetryRepo,
    saved_task_repo: SavedTaskRepo,
    worker_handle: TemporalWorkerHandle,
):

    app = init_rest_server(
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

    def _run():
        uvicorn.run(app, host="0.0.0.0", port=3000)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
