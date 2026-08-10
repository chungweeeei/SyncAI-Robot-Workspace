"""Tests for the WorkflowGateway's per-robot scope on the shared namespace.

Task queues are per-robot but the Temporal namespace is shared, so workflow ids
and schedule ids are global. What is pinned here is the ownership gate that
keeps one robot's backend from reading, cancelling, pausing or deleting another
robot's work through nothing more than an id:

* ``get_task_state`` / ``cancel_task`` verify the described execution's
  ``task_queue`` and answer 404 for a foreign one — 404 rather than 403 so a
  caller who addressed the wrong robot learns nothing about the id's existence
  (the map router's ``_require_vertex`` precedent).
* Single-schedule verbs go through ``_require_owned_schedule``, which reads the
  ownership fact off the *describe* path's full action — no memo needed, so it
  holds for legacy schedules too.
* ``list_schedules`` scopes on the memo's ``robot_id`` (the list path cannot
  see the task queue), with a describe-once-and-cache fallback for legacy rows
  whose memo predates the key.

The Temporal client is stubbed at ``gateway._client``, which ``_get_client``
returns as-is once set — the same seam the real code uses for its lazy connect.
No pytest-asyncio here; the async surface is exercised with ``asyncio.run``.
"""

import asyncio

import pytest

pytest.importorskip("temporalio")

from types import SimpleNamespace  # noqa: E402

from temporalio.client import (  # noqa: E402
    ScheduleActionStartWorkflow,
    WorkflowExecutionStatus,
)

from syncai_backend.exceptions import NotFoundError  # noqa: E402
from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    ScheduleTask,
    ScheduleTrigger,
    WorkflowTaskDefinition,
)
from syncai_backend.gateways.workflow.workflow import (  # noqa: E402
    WorkflowGateway,
    _schedule_task_queue,
    _schedule_to_memo,
)


OWN_QUEUE = "robot01.ROBOT_TASK_QUEUE"
FOREIGN_QUEUE = "robot02.ROBOT_TASK_QUEUE"


def _start_workflow_action(task_queue: str) -> ScheduleActionStartWorkflow:
    return ScheduleActionStartWorkflow(
        "RobotWorkflow", args=[], id="sched-1", task_queue=task_queue
    )


def _schedule_desc(action) -> SimpleNamespace:
    """A described schedule, reduced to what _schedule_task_queue reads."""
    return SimpleNamespace(schedule=SimpleNamespace(action=action))


class _StubWorkflowHandle:
    def __init__(self, description=None, steps=None):
        self._description = description
        self._steps = steps if steps is not None else []
        self.cancelled = False

    async def describe(self):
        return self._description

    async def query(self, name, result_type=None):
        return self._steps

    async def cancel(self):
        self.cancelled = True


class _StubScheduleHandle:
    def __init__(self, description=None, error=None):
        self._description = description
        self._error = error
        self.describe_calls = 0

    async def describe(self):
        self.describe_calls += 1
        if self._error is not None:
            raise self._error
        return self._description


class _StubClient:
    def __init__(self, workflow_handle=None, schedule_handle=None):
        self._workflow_handle = workflow_handle
        self._schedule_handle = schedule_handle

    def get_workflow_handle(self, task_id):
        return self._workflow_handle

    def get_schedule_handle(self, schedule_id):
        return self._schedule_handle


def _gateway(logger, client=None) -> WorkflowGateway:
    gw = WorkflowGateway(logger=logger, robot_id="robot01")
    gw._client = client
    return gw


# --- The module-level helpers -------------------------------------------------


def test_schedule_task_queue_reads_the_full_action():
    desc = _schedule_desc(_start_workflow_action(OWN_QUEUE))
    assert _schedule_task_queue(desc) == OWN_QUEUE


def test_schedule_task_queue_is_none_for_the_list_shape():
    # A ScheduleListActionStartWorkflow carries the workflow type name and
    # nothing else; anything that is not the full action must answer None so
    # ownership degrades closed.
    desc = _schedule_desc(SimpleNamespace(workflow="RobotWorkflow"))
    assert _schedule_task_queue(desc) is None


def test_schedule_task_queue_is_none_without_an_action():
    assert _schedule_task_queue(SimpleNamespace(schedule=SimpleNamespace())) is None


def test_schedule_memo_always_carries_robot_id():
    schedule = ScheduleTask(
        id="sched-1",
        trigger=ScheduleTrigger(interval_seconds=60),
        definition=WorkflowTaskDefinition(steps=[]),
    )
    memo = _schedule_to_memo(schedule, "robot01")
    assert memo["robot_id"] == "robot01"
    assert memo["interval_seconds"] == 60


# --- Task ownership -----------------------------------------------------------


def test_get_task_state_hides_a_foreign_task(logger):
    handle = _StubWorkflowHandle(
        description=SimpleNamespace(
            status=WorkflowExecutionStatus.RUNNING, task_queue=FOREIGN_QUEUE
        )
    )
    gw = _gateway(logger, _StubClient(workflow_handle=handle))

    with pytest.raises(NotFoundError):
        asyncio.run(gw.get_task_state("robot02-task"))


def test_get_task_state_answers_for_its_own_task(logger):
    handle = _StubWorkflowHandle(
        description=SimpleNamespace(
            status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
        )
    )
    gw = _gateway(logger, _StubClient(workflow_handle=handle))

    state = asyncio.run(gw.get_task_state("robot01-task"))
    assert state.status == "IN_PROGRESS"


def test_cancel_task_refuses_a_foreign_task(logger):
    # The load-bearing half: the foreign task is not merely hidden, the cancel
    # RPC is never issued.
    handle = _StubWorkflowHandle(
        description=SimpleNamespace(
            status=WorkflowExecutionStatus.RUNNING, task_queue=FOREIGN_QUEUE
        )
    )
    gw = _gateway(logger, _StubClient(workflow_handle=handle))

    with pytest.raises(NotFoundError):
        asyncio.run(gw.cancel_task("robot02-task"))
    assert handle.cancelled is False


def test_cancel_task_cancels_its_own_task(logger):
    handle = _StubWorkflowHandle(
        description=SimpleNamespace(
            status=WorkflowExecutionStatus.RUNNING, task_queue=OWN_QUEUE
        )
    )
    gw = _gateway(logger, _StubClient(workflow_handle=handle))

    asyncio.run(gw.cancel_task("robot01-task"))
    assert handle.cancelled is True


# --- Schedule ownership (describe path) ----------------------------------------


def test_require_owned_schedule_hides_a_foreign_schedule(logger):
    handle = _StubScheduleHandle(
        description=_schedule_desc(_start_workflow_action(FOREIGN_QUEUE))
    )
    client = _StubClient(schedule_handle=handle)
    gw = _gateway(logger, client)

    with pytest.raises(NotFoundError):
        asyncio.run(gw._require_owned_schedule(client, "robot02-sched"))


def test_require_owned_schedule_passes_its_own(logger):
    desc = _schedule_desc(_start_workflow_action(OWN_QUEUE))
    client = _StubClient(schedule_handle=_StubScheduleHandle(description=desc))
    gw = _gateway(logger, client)

    assert asyncio.run(gw._require_owned_schedule(client, "robot01-sched")) is desc


# --- Schedule ownership (list path) ---------------------------------------------


def test_list_owner_trusts_the_memo_without_an_rpc(logger):
    # A client whose describe would blow up proves the memo path never asks.
    boom = _StubScheduleHandle(error=RuntimeError("describe must not be called"))
    client = _StubClient(schedule_handle=boom)
    gw = _gateway(logger, client)

    assert asyncio.run(
        gw._owns_listed_schedule(client, "s1", {"robot_id": "robot01"})
    )
    assert not asyncio.run(
        gw._owns_listed_schedule(client, "s2", {"robot_id": "robot02"})
    )
    assert boom.describe_calls == 0


def test_list_owner_resolves_a_legacy_row_once_and_caches(logger):
    handle = _StubScheduleHandle(
        description=_schedule_desc(_start_workflow_action(OWN_QUEUE))
    )
    client = _StubClient(schedule_handle=handle)
    gw = _gateway(logger, client)

    assert asyncio.run(gw._owns_listed_schedule(client, "legacy", {}))
    assert asyncio.run(gw._owns_listed_schedule(client, "legacy", {}))
    # Second answer came from the cache, not a second RPC.
    assert handle.describe_calls == 1


def test_list_owner_hides_a_legacy_row_on_error_without_caching(logger):
    failing = _StubClient(
        schedule_handle=_StubScheduleHandle(error=RuntimeError("rpc down"))
    )
    working = _StubClient(
        schedule_handle=_StubScheduleHandle(
            description=_schedule_desc(_start_workflow_action(OWN_QUEUE))
        )
    )
    gw = _gateway(logger, failing)

    # Hidden this round, but the failure must not stick to the schedule:
    # the next round, with the RPC back, resolves it as owned.
    assert not asyncio.run(gw._owns_listed_schedule(failing, "legacy", {}))
    assert asyncio.run(gw._owns_listed_schedule(working, "legacy", {}))
