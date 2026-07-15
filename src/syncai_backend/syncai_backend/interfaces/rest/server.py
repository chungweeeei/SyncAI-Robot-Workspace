import threading
import uvicorn
import structlog
from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from syncai_backend.exceptions import (
    BadRequestError,
    InternalServerError,
    NotFoundError,
    UnauthorizedError,
)

from syncai_backend.interfaces.rest.routers.task import init_task_router
from syncai_backend.interfaces.rest.routers.schedule import init_schedule_router
from syncai_backend.interfaces.rest.routers.robot import init_robot_router
from syncai_backend.interfaces.rest.routers.network import init_network_router
from syncai_backend.interfaces.rest.routers.map import init_map_router

from syncai_backend.repositories.robot.robot import RobotRepo
from syncai_backend.repositories.map.map import MapRepo

from syncai_backend.gateways.workflow.workflow import WorkflowGateway
from syncai_backend.gateways.robot.robot import RobotGateway


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

    app.include_router(init_task_router(logger=logger, workflow_gw=workflow_gw))
    app.include_router(init_schedule_router(logger=logger, workflow_gw=workflow_gw))
    app.include_router(init_robot_router(logger=logger, robot_repo=robot_repo))
    app.include_router(init_network_router(logger=logger, robot_gw=robot_gw))
    app.include_router(init_map_router(logger=logger, map_repo=map_repo))

    return app


def start_rest_server(
    logger: structlog.stdlib.BoundLogger,
    workflow_gw: WorkflowGateway,
    robot_repo: RobotRepo,
    robot_gw: RobotGateway,
    map_repo: MapRepo,
):

    app = init_rest_server(
        logger=logger,
        workflow_gw=workflow_gw,
        robot_repo=robot_repo,
        robot_gw=robot_gw,
        map_repo=map_repo,
    )

    def _run():
        uvicorn.run(app, host="0.0.0.0", port=3000)

    thread = threading.Thread(target=_run, daemon=True)
    thread.start()
