"""Tests for the Temporal worker's bounded connect-retry and its /health signal.

Before this, a failed ``Client.connect`` killed the worker's daemon thread
silently: ``ready.wait(timeout=10)`` elapsed, POST /tasks kept answering 200
(the gateway lazy-connects), and the queued work never ran. What is pinned
here is that every exit path of the worker thread now lands in the
``TemporalWorkerHandle`` — connect retried then running, retry budget
exhausted then dead, ``worker.run()`` dying mid-flight then dead — and that
/health projects the handle as the operator-facing ``task_server`` field
(never a non-200; the endpoint stays a liveness probe).

``Client`` and ``Worker`` are stubbed at the worker module's namespace, not
patched on the temporalio classes themselves, so the real SDK is never
touched. No pytest-asyncio; the async surface is exercised with
``asyncio.run``.
"""

import asyncio
import threading
from unittest.mock import MagicMock

import pytest
import structlog

pytest.importorskip("temporalio")

from syncai_backend.temporal import worker as worker_mod  # noqa: E402
from syncai_backend.temporal.worker import (  # noqa: E402
    TemporalWorkerHandle,
    run_worker,
    start_temporal_worker,
)

logger = structlog.get_logger()


class _StubClient:
    """Fails ``connect`` a configured number of times, then succeeds."""

    failures = 0
    calls = 0

    @classmethod
    async def connect(cls, *args, **kwargs):
        cls.calls += 1
        if cls.calls <= cls.failures:
            raise ConnectionError("Connection refused (stub)")
        return object()

    @classmethod
    def reset(cls, failures: int) -> None:
        cls.failures = failures
        cls.calls = 0


class _StubWorker:
    run_error: Exception | None = None

    def __init__(self, *args, **kwargs):
        pass

    async def run(self):
        if _StubWorker.run_error is not None:
            raise _StubWorker.run_error


@pytest.fixture(autouse=True)
def _stub_temporal(monkeypatch):
    _StubClient.reset(failures=0)
    _StubWorker.run_error = None
    monkeypatch.setattr(worker_mod, "Client", _StubClient)
    monkeypatch.setattr(worker_mod, "Worker", _StubWorker)
    monkeypatch.setattr(worker_mod, "RETRY_INTERVAL", 0)


def _activities():
    # run_worker only forwards the four activity methods into Worker, which is
    # stubbed here, so a MagicMock quacks well enough.
    return MagicMock()


def test_connect_retries_then_runs():
    _StubClient.reset(failures=2)
    handle = TemporalWorkerHandle()
    ready = threading.Event()

    asyncio.run(
        run_worker(
            logger, robot_id="robot01", activities=_activities(), handle=handle, ready=ready
        )
    )

    assert _StubClient.calls == 3
    assert handle.snapshot() == (TemporalWorkerHandle.STATUS_RUNNING, None)
    assert ready.is_set()


def test_connect_exhaustion_marks_dead_without_raising(monkeypatch):
    monkeypatch.setattr(worker_mod, "MAX_RETRIES", 3)
    _StubClient.reset(failures=100)
    handle = TemporalWorkerHandle()
    ready = threading.Event()

    # Must return, not raise — in production a raise here is exactly the
    # silent thread death this module exists to prevent.
    asyncio.run(
        run_worker(
            logger, robot_id="robot01", activities=_activities(), handle=handle, ready=ready
        )
    )

    assert _StubClient.calls == 3
    status, error = handle.snapshot()
    assert status == TemporalWorkerHandle.STATUS_DEAD
    assert "Connection refused" in error
    assert not ready.is_set()


def test_worker_run_dying_marks_dead():
    # Through start_temporal_worker, so the thread-level catch is what is
    # exercised: worker.run() raising escapes asyncio.run and must land in
    # the handle instead of vanishing with the thread.
    _StubWorker.run_error = RuntimeError("poller exploded")

    handle = start_temporal_worker(logger, robot_id="robot01", robot_gw=MagicMock())
    handle.thread.join(timeout=10)

    assert not handle.thread.is_alive()
    status, error = handle.snapshot()
    assert status == TemporalWorkerHandle.STATUS_DEAD
    assert "poller exploded" in error


def test_health_projects_worker_state():
    pytest.importorskip("httpx")
    from fastapi.testclient import TestClient

    from syncai_backend.interfaces.rest.server import init_rest_server

    handle = TemporalWorkerHandle()
    app = init_rest_server(
        logger=logger,
        workflow_gw=MagicMock(),
        robot_repo=MagicMock(),
        robot_gw=MagicMock(),
        map_repo=MagicMock(),
        map_catalog_repo=MagicMock(),
        map_gw=MagicMock(),
        pointcloud_repo=MagicMock(),
        telemetry_repo=MagicMock(),
        saved_task_repo=MagicMock(),
        worker_handle=handle,
    )
    client = TestClient(app)

    # Still connecting: degraded, but HTTP 200 — /health stays a liveness
    # probe; a 503 wired into a container healthcheck would restart-loop the
    # backend whenever Temporal is down for long.
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {
        "status": "degraded",
        "task_server": "connecting",
        "task_server_error": None,
    }

    handle.mark_running()
    assert client.get("/health").json() == {
        "status": "ok",
        "task_server": "running",
        "task_server_error": None,
    }

    handle.mark_dead("Connection refused (stub)")
    response = client.get("/health")
    assert response.json()["status"] == "degraded"
    assert response.json()["task_server"] == "dead"
    assert "Connection refused" in response.json()["task_server_error"]
