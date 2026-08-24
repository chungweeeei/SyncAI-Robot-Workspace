"""Tests for ``RobotWorkflow``, run against the time-skipping test server.

Unlike the activity tests (``test_activities.py``, ``ActivityEnvironment``)
and the gateway tests (client mocked at ``Client.connect``), these run the
*real* workflow code under ``WorkflowEnvironment.start_time_skipping()``: a
real Temporal test server, a real ``Worker``, the real retry policy. What is
mocked is only the activity layer — the workflow schedules activities by
function reference, but all that crosses the wire is the ``@activity.defn``
*name*, so a local ``async def`` registered under the same name is the exact
seam a production worker presents. Activity internals stay pinned in
``test_activities.py``; here they are stand-ins that record, gate, or fail.

What is pinned:

* Steps run strictly in order and their pydantic params survive the round
  trip through the pydantic data converter (the worker's converter is not
  the default one — forgetting it on the client is a real integration bug).
* ``get_step_states`` reports per-step status mid-run — a gated activity
  holds the workflow at a known point — and remains queryable after the
  workflow has closed, which is exactly how the REST layer reads the state
  of a finished task.
* A failed step fails the whole run: the failing step carries the cause in
  ``error_msg``, later steps stay PENDING and their activities never run.
  Both failure shapes are covered — the activity raising, and the activity
  returning ``success=False`` — because the workflow reports them with
  different messages.
* The retry policy is 3 attempts of the *same* step, not a re-run of the
  workflow. Time skipping is what makes the two 5 s backoffs free.
* ``handle.cancel()`` closes the run as canceled, and only the in-flight
  step is touched — but that step ends FAILED / "Cancelled", *not* the
  CANCELED / "Task canceled" the workflow's ``except asyncio.CancelledError``
  branch would write. See the cancellation test for why that branch is
  bypassed; this suite pins what actually happens, deliberately.

Deliberately not covered: the unknown-step-type guard in the dispatch table.
``Step`` validation rejects anything outside ``StepType`` before a task can
reach the workflow, so that branch is unreachable through the public surface;
pinning it would mean forging an invalid Step.

Mechanics, matching the register of the other suites: no pytest-asyncio
(pytest 6.2.5, no plugins), so each test is a sync function wrapping one
``asyncio.run``. Every test boots its own environment — the test-server
binary is downloaded once and cached, after which a boot costs ~0.1 s, which
is cheaper than sharing one event loop across tests would be worth.
"""

import asyncio
from contextlib import asynccontextmanager

import pytest

pytest.importorskip("temporalio")

from temporalio import activity  # noqa: E402
from temporalio.client import WorkflowFailureError  # noqa: E402
from temporalio.contrib.pydantic import pydantic_data_converter  # noqa: E402
from temporalio.exceptions import (  # noqa: E402
    ActivityError,
    ApplicationError,
    CancelledError,
)
from temporalio.testing import WorkflowEnvironment  # noqa: E402
from temporalio.worker import Worker  # noqa: E402

from syncai_backend.gateways.workflow.schema import (  # noqa: E402
    MoveParams,
    Step,
    StepStatus,
    StepType,
    WorkflowTask,
    WorkflowTaskDefinition,
)
from syncai_backend.temporal.activities import ActivityResult  # noqa: E402
from syncai_backend.temporal.workflows import RobotWorkflow  # noqa: E402

# Same shape the production worker derives from robot_id; the value itself is
# arbitrary here since client and worker share it within one test.
TASK_QUEUE = "robot01.ROBOT_TASK_QUEUE"


def _task(task_id: str, *step_types: StepType) -> WorkflowTask:
    steps = [
        Step(
            id=f"step-{i}",
            type=step_type,
            # Schema contract: MOVE carries params, the posture steps carry
            # none (Step validation enforces it, the workflow relies on it
            # for its empty-args special case).
            params=(
                MoveParams(x=1.5, y=-2.5, theta=90.0)
                if step_type is StepType.MOVE
                else None
            ),
        )
        for i, step_type in enumerate(step_types)
    ]
    return WorkflowTask(id=task_id, definition=WorkflowTaskDefinition(steps=steps))


def _instant_activities(calls: list):
    """The three activity names the workflow dispatches to, succeeding at once.

    Each records ``(name, arg)`` so a test can pin ordering and the params
    hand-off without ever leaving the happy path.
    """

    @activity.defn(name="execute_move")
    async def execute_move(params: MoveParams) -> ActivityResult:
        calls.append(("execute_move", params))
        return ActivityResult(success=True, goal_id="goal-1", state="succeeded")

    @activity.defn(name="execute_stand")
    async def execute_stand() -> ActivityResult:
        calls.append(("execute_stand", None))
        return ActivityResult(success=True, state="succeeded")

    @activity.defn(name="execute_lie_down")
    async def execute_lie_down() -> ActivityResult:
        calls.append(("execute_lie_down", None))
        return ActivityResult(success=True, state="succeeded")

    return [execute_move, execute_stand, execute_lie_down]


def _gated_move(started: asyncio.Event, release: asyncio.Event):
    """A MOVE activity that parks until released, heartbeating while it waits.

    The heartbeats are load-bearing twice over: the workflow schedules every
    activity with ``heartbeat_timeout=3s`` (real time — the test server does
    not skip while an activity is outstanding), and a cancel is only ever
    delivered to a running activity in a heartbeat response.
    """

    @activity.defn(name="execute_move")
    async def execute_move(params: MoveParams) -> ActivityResult:
        started.set()
        while not release.is_set():
            activity.heartbeat()
            try:
                await asyncio.wait_for(release.wait(), timeout=0.2)
            except asyncio.TimeoutError:
                pass
        return ActivityResult(success=True, state="succeeded")

    return execute_move


@asynccontextmanager
async def _worker(activities):
    """A time-skipping environment with a worker polling TASK_QUEUE.

    The client is built with ``pydantic_data_converter`` to mirror
    ``run_worker`` — the default converter cannot serialize WorkflowTask.
    """
    env = await WorkflowEnvironment.start_time_skipping(
        data_converter=pydantic_data_converter
    )
    try:
        async with Worker(
            env.client,
            task_queue=TASK_QUEUE,
            workflows=[RobotWorkflow],
            activities=activities,
        ):
            yield env.client
    finally:
        await env.shutdown()


async def _start(client, task: WorkflowTask):
    return await client.start_workflow(
        RobotWorkflow.run, task, id=task.id, task_queue=TASK_QUEUE
    )


def test_steps_run_in_order_and_all_complete():
    async def scenario():
        calls: list = []
        task = _task("task-happy", StepType.MOVE, StepType.STANDUP, StepType.LIEDOWN)

        async with _worker(_instant_activities(calls)) as client:
            handle = await _start(client, task)
            await handle.result()
            return calls, await handle.query(RobotWorkflow.get_step_states)

    calls, steps = asyncio.run(scenario())

    # One activity per step, in definition order — the whole point of the
    # sequential loop over task.definition.steps.
    assert [name for name, _ in calls] == [
        "execute_move",
        "execute_stand",
        "execute_lie_down",
    ]
    # The MOVE params must survive the pydantic converter round trip intact
    # (still degrees here; the degrees→radians conversion is the activity's
    # job, pinned in test_activities.py).
    move_params = calls[0][1]
    assert (move_params.x, move_params.y, move_params.theta) == (1.5, -2.5, 90.0)
    assert [step.status for step in steps] == [StepStatus.COMPLETED] * 3


def test_query_reports_per_step_state_mid_run():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        calls: list = []
        # Reuse the instant stand/lie_down mocks; only MOVE is gated.
        activities = [_gated_move(started, release)] + _instant_activities(calls)[1:]
        task = _task("task-midrun", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            mid = await handle.query(RobotWorkflow.get_step_states)
            release.set()
            await handle.result()
            done = await handle.query(RobotWorkflow.get_step_states)
            return mid, done

    mid, done = asyncio.run(scenario())

    # The workflow flips a step to IN_PROGRESS before scheduling its
    # activity, so once the activity has started the query must say so —
    # this is what the operator console polls while a task runs.
    assert [step.status for step in mid] == [
        StepStatus.IN_PROGRESS,
        StepStatus.PENDING,
    ]
    assert [step.status for step in done] == [StepStatus.COMPLETED] * 2


def test_activity_failure_fails_the_step_and_halts_the_run():
    async def scenario():
        calls: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            # non_retryable so the run fails on the first attempt; the
            # retryable shape has its own test below.
            raise ApplicationError("nav server says no", non_retryable=True)

        activities = [execute_move] + _instant_activities(calls)[1:]
        task = _task("task-failing", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return calls, exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    calls, err, steps = asyncio.run(scenario())

    # The workflow re-raises the ActivityError itself, so the failure chain
    # keeps the activity's own error as the innermost cause.
    assert isinstance(err.cause, ActivityError)
    assert isinstance(err.cause.cause, ApplicationError)
    assert steps[0].status is StepStatus.FAILED
    # error_msg carries the *cause* (the activity's message), not the
    # ActivityError wrapper's generic text.
    assert "nav server says no" in steps[0].error_msg
    # Failure halts the run: the next step is untouched and its activity
    # never executed.
    assert steps[1].status is StepStatus.PENDING
    assert calls == []


def test_unsuccessful_result_fails_the_workflow():
    async def scenario():
        @activity.defn(name="execute_stand")
        async def execute_stand() -> ActivityResult:
            # The activity completes normally but reports failure — the
            # other failure shape, distinct from raising. Today's activities
            # never actually return success=False (they raise instead), but
            # the workflow guards it, so pin what the guard does.
            return ActivityResult(success=False)

        task = _task("task-unsuccessful", StepType.STANDUP)

        async with _worker([execute_stand]) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    err, steps = asyncio.run(scenario())

    assert isinstance(err.cause, ApplicationError)
    assert err.cause.non_retryable is True
    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == "activity failed"


def test_a_retryable_failure_gets_three_attempts_of_the_same_step():
    async def scenario():
        attempts: list = []

        @activity.defn(name="execute_move")
        async def execute_move(params: MoveParams) -> ActivityResult:
            attempts.append(activity.info().attempt)
            raise ApplicationError("move ended in aborted", non_retryable=False)

        task = _task("task-retries", StepType.MOVE)

        async with _worker([execute_move]) as client:
            handle = await _start(client, task)
            with pytest.raises(WorkflowFailureError):
                # Time skipping fast-forwards the retry backoff (5 s initial
                # interval) — this await is where the skipping happens.
                await handle.result()
            return attempts, await handle.query(RobotWorkflow.get_step_states)

    attempts, steps = asyncio.run(scenario())

    # maximum_attempts=3 in the workflow's RetryPolicy: exactly three tries
    # of the one step (Temporal-side retry, invisible to the step list), then
    # a single FAILED step — not an eternal IN_PROGRESS, which is the bug the
    # policy exists to prevent.
    assert attempts == [1, 2, 3]
    assert steps[0].status is StepStatus.FAILED


def test_cancellation_ends_the_run_canceled_and_fails_the_in_flight_step():
    async def scenario():
        started, release = asyncio.Event(), asyncio.Event()
        calls: list = []
        # release is never set: the MOVE parks until the cancel arrives.
        activities = [_gated_move(started, release)] + _instant_activities(calls)[1:]
        task = _task("task-canceled", StepType.MOVE, StepType.STANDUP)

        async with _worker(activities) as client:
            handle = await _start(client, task)
            await asyncio.wait_for(started.wait(), timeout=10)
            await handle.cancel()
            with pytest.raises(WorkflowFailureError) as exc_info:
                await handle.result()
            return calls, exc_info.value, await handle.query(RobotWorkflow.get_step_states)

    calls, err, steps = asyncio.run(scenario())

    # The run itself closes as canceled: WorkflowFailureError with a bare
    # CancelledError cause is the canceled-workflow shape, not the failed one.
    assert isinstance(err.cause, CancelledError)
    # The in-flight step, however, ends FAILED / "Cancelled" — NOT the
    # CANCELED / "Task canceled" that the workflow's
    # ``except asyncio.CancelledError`` branch would write. Under SDK 1.31
    # with WAIT_CANCELLATION_COMPLETED, the cancelled activity surfaces from
    # ``workflow.execute_activity`` as ActivityError(cause=CancelledError),
    # so the ``except ActivityError`` branch runs first and the
    # asyncio.CancelledError branch is unreachable on this path. Pinned
    # as-observed rather than "fixed" in the test: if the workflow is ever
    # amended to catch the ActivityError-wrapped cancel (so the console can
    # tell "canceled" from "failed"), this is the assertion that should
    # break.
    assert steps[0].status is StepStatus.FAILED
    assert steps[0].error_msg == "Cancelled"
    # The step that never started stays PENDING and its activity never ran —
    # "interrupted here" and "never got there" must stay distinguishable.
    assert steps[1].status is StepStatus.PENDING
    assert calls == []
