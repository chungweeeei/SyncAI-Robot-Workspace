import structlog
from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from exceptions import (
    BadRequestError,
    InternalServerError,
    NotFoundError,
    UnauthorizedError,
)


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


def init_rest_server(logger: structlog.stdlib.BoundLogger) -> FastAPI:

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

    # app.include_router(init_task_router(logger=logger))
