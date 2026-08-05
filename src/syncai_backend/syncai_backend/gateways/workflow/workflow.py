import asyncio
import structlog
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import List, Optional, Tuple

from temporalio.contrib.pydantic import pydantic_data_converter
from temporalio.client import (
    Client,
    WorkflowExecutionStatus,
    Schedule,
    ScheduleActionStartWorkflow,
    ScheduleAlreadyRunningError,
    ScheduleIntervalSpec,
    ScheduleOverlapPolicy,
    SchedulePolicy,
    ScheduleSpec,
)
from temporalio.api.common.v1 import Payload
from temporalio.common import SearchAttributeKey
from temporalio.service import RPCError, RPCStatusCode

from syncai_backend.exceptions import (
    BadRequestError,
    InternalServerError,
    NotFoundError,
)

from syncai_backend.temporal.shared import TEMPORAL_SERVER_URL

from syncai_backend.gateways.workflow.schema import (
    ActiveTask,
    ScheduleTask,
    ScheduleTrigger,
    ScheduleView,
    Step,
    TaskSource,
    TaskState,
    WorkflowTask,
)
from syncai_backend.gateways.workflow.config import (
    ACTIVE_TASK_CACHE_TTL_S,
    ACTIVE_TASK_LIST_LIMIT,
    ACTIVE_TASK_RPC_TIMEOUT_S,
    WORKFLOW_TYPE_NAME,
)


_WORKFLOW_STATUS_MAP = {
    WorkflowExecutionStatus.RUNNING: "IN_PROGRESS",
    WorkflowExecutionStatus.CONTINUED_AS_NEW: "IN_PROGRESS",
    WorkflowExecutionStatus.COMPLETED: "COMPLETED",
    WorkflowExecutionStatus.FAILED: "FAILED",
    WorkflowExecutionStatus.TIMED_OUT: "FAILED",
    WorkflowExecutionStatus.CANCELED: "CANCELED",
    WorkflowExecutionStatus.TERMINATED: "CANCELED",
}


# The schedule that started a run, as Temporal itself records it.
#
# This is a *predefined* search attribute, written by the schedule machinery on
# every triggered execution — this codebase sets no custom search attributes and
# does not need to. It is also the only way the provenance is recoverable at
# all: a scheduled run's workflow id is `<schedule_id>-<nominal ISO time>`, a
# string the backend never forms, never stores and cannot reconstruct (it would
# have to guess the exact nominal instant). Nothing in the memo helps either —
# the memo belongs to the *schedule*, not to the run it starts.
_SCHEDULED_BY_KEY = SearchAttributeKey.for_keyword("TemporalScheduledById")


def _active_query(task_queue: str) -> str:
    """The visibility List Filter behind GET /api/v1/active_tasks.

    `TaskQueue` is the per-robot scope, so no custom attribute is needed: every
    execution for this robot is enqueued on `<robot_id>.ROBOT_TASK_QUEUE` by
    WorkflowGateway, whoever started it. `WorkflowType` keeps a future second
    workflow type on the same queue out of the answer.

    Requires SQL advanced visibility, i.e. Temporal server >= 1.20 on Postgres
    or MySQL — the compose stack pins temporalio/auto-setup:1.29.7 on Postgres,
    so this is satisfied. A deployment on standard visibility would reject the
    query with INVALID_ARGUMENT; see the handler in _fetch_active_tasks for what
    to do about it.
    """
    return (
        f"WorkflowType = '{WORKFLOW_TYPE_NAME}' "
        f"AND TaskQueue = '{task_queue}' "
        "AND ExecutionStatus = 'Running'"
    )


def _schedule_id_of(execution) -> Optional[str]:
    """The schedule that started this run, or None for a direct dispatch."""
    try:
        return execution.typed_search_attributes.get(_SCHEDULED_BY_KEY)
    except Exception:
        # Never fatal, same policy as _read_steps / _read_memo: provenance is
        # decoration, presence is the safety fact. A run whose attributes cannot
        # be decoded must still be reported as running.
        return None


@dataclass
class _ActiveSnapshot:
    """One cached answer — success or failure — and when it was taken.

    `fetched_at` is a monotonic reading (TTL arithmetic must not be affected by
    a wall-clock step), while `as_of` is the wall clock the client renders
    elapsed times against.
    """

    fetched_at: float
    as_of: datetime
    tasks: Optional[List[ActiveTask]] = None
    error: Optional[Exception] = None


def _build_schedule_spec(trigger: ScheduleTrigger) -> ScheduleSpec:
    """Map a ScheduleTrigger (cron or interval) to a Temporal ScheduleSpec."""
    if trigger.cron:
        return ScheduleSpec(
            cron_expressions=[trigger.cron],
            time_zone_name=trigger.timezone or "",
        )
    if trigger.interval_seconds:
        return ScheduleSpec(
            intervals=[
                ScheduleIntervalSpec(every=timedelta(seconds=trigger.interval_seconds))
            ]
        )
    raise BadRequestError("Schedule trigger must set either cron or intervalSeconds")


def _spec_to_trigger(spec: ScheduleSpec) -> ScheduleTrigger:
    """Reconstruct a ScheduleTrigger from a Temporal ScheduleSpec.

    Fallback for schedules not created through this API. Note that Temporal
    normalises cron_expressions into calendar specs, so a cron created
    elsewhere will not round-trip here -- see _read_trigger / the memo path.
    """
    if spec.cron_expressions:
        return ScheduleTrigger(
            cron=spec.cron_expressions[0],
            timezone=spec.time_zone_name or None,
        )
    if spec.intervals:
        return ScheduleTrigger(
            interval_seconds=int(spec.intervals[0].every.total_seconds())
        )
    return ScheduleTrigger()


def _schedule_to_memo(schedule: ScheduleTask) -> dict:
    """Serialise a schedule's trigger and provenance into memo fields.

    The trigger has to be here because Temporal normalises cron_expressions into
    internal calendar specs, so describe() can no longer report the string that
    was registered. The provenance rides along in the same place for a different
    reason: the memo is readable from ``list_schedules()`` while the
    start-workflow args are not, so this is the only channel by which the
    collection endpoint can say which map a schedule belongs to.
    """
    memo: dict = {}
    if schedule.trigger.cron:
        memo["cron"] = schedule.trigger.cron
    if schedule.trigger.interval_seconds:
        memo["interval_seconds"] = schedule.trigger.interval_seconds
    if schedule.trigger.timezone:
        memo["timezone"] = schedule.trigger.timezone
    if schedule.map_name:
        memo["map_name"] = schedule.map_name
    if schedule.saved_task_id:
        memo["saved_task_id"] = schedule.saved_task_id
    if schedule.saved_task_name:
        memo["saved_task_name"] = schedule.saved_task_name
    return memo


async def _read_memo(described) -> dict:
    """One memo read per description, tolerating a schedule that has none.

    Split out from the interpretation below so each call site awaits the decode
    exactly once and then pulls both the trigger and the provenance out of the
    result -- the previous shape awaited it inside the trigger helper, which
    would have meant a second decode per row once provenance was added.
    """
    try:
        return await described.memo() or {}
    except Exception:
        return {}


def _trigger_from_memo(memo: dict, spec: ScheduleSpec) -> ScheduleTrigger:
    """The trigger as registered, falling back to the (normalised) spec."""
    cron = memo.get("cron")
    interval_seconds = memo.get("interval_seconds")

    if cron or interval_seconds:
        return ScheduleTrigger(
            cron=cron,
            interval_seconds=interval_seconds,
            timezone=memo.get("timezone"),
        )

    return _spec_to_trigger(spec)


async def _read_steps(logger: structlog.stdlib.BoundLogger, desc) -> List[Step]:
    """Decode the frozen step list out of a *described* schedule's action.

    ``ScheduleActionStartWorkflow._from_proto`` leaves ``args`` as the raw
    ``temporalio.api.common.v1.Payload`` protos, and the ScheduleDescription
    carries the data_converter that turns them back into a WorkflowTask.

    Module-level, and taking the logger rather than reading ``self._logger``, so
    it can be unit-tested against a hand-built description -- the same reason
    ``_spec_to_trigger`` is not a method.

    Never fatal. A schedule created outside this API, or one whose payload this
    converter cannot map, must still report its trigger and next run times --
    the same policy ``get_task_state``'s step query and the memo read above use.
    """
    action = getattr(desc.schedule, "action", None)
    if not isinstance(action, ScheduleActionStartWorkflow) or not action.args:
        return []

    try:
        args = list(action.args)
        # Guarded rather than assumed: args are Payloads only on the describe
        # path. A ScheduleTask this process just built still holds the python
        # objects, and so does a test's stub.
        if isinstance(args[0], Payload):
            args = await desc.data_converter.decode(args, [WorkflowTask])
        task = args[0]
        return list(task.definition.steps) if isinstance(task, WorkflowTask) else []
    except Exception as err:
        logger.warn(
            "[WorkflowGateway] Failed to decode schedule steps",
            schedule_id=getattr(desc, "id", None),
            error=str(err),
        )
        return []


class WorkflowGateway:
    def __init__(self, logger: structlog.stdlib.BoundLogger, robot_id: str):
        self._logger = logger
        # Tasks are enqueued on this robot's own task queue (must match the
        # worker's queue name in temporal/worker.py) so its own Temporal
        # worker — not another robot's — executes them.
        self._task_queue = f"{robot_id}.ROBOT_TASK_QUEUE"
        self._client: Client | None = None

        # The active-task cache. Both fields are touched ONLY from the uvicorn
        # event loop (start_rest_server runs uvicorn on its own daemon thread);
        # the Temporal worker in temporal/worker.py runs asyncio.run on a
        # different thread with its own Client and must never reach this.
        #
        # The lock is created lazily rather than here because this gateway is
        # constructed on the main thread, before that loop exists.
        self._active_snapshot: Optional[_ActiveSnapshot] = None
        self._active_lock: Optional[asyncio.Lock] = None

    async def _get_client(self) -> Client:
        if self._client is not None:
            return self._client

        try:
            self._client = await Client.connect(
                target_host=TEMPORAL_SERVER_URL,
                data_converter=pydantic_data_converter,
            )
        except Exception as err:
            raise err

        return self._client

    async def start_task(self, request: WorkflowTask):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        try:
            await client.start_workflow(
                workflow=WORKFLOW_TYPE_NAME,
                id=request.id,
                args=[request],
                task_queue=self._task_queue,
            )
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to start workflow", error=str(err)
            )
            raise InternalServerError("Start workflow failed")

    async def get_task_state(self, task_id: str) -> TaskState:

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_workflow_handle(task_id)

        try:
            description = await handle.describe()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Task {task_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to describe workflow", error=str(err)
            )
            raise InternalServerError("Describe workflow failed")

        status = _WORKFLOW_STATUS_MAP.get(description.status)
        if status is None:
            self._logger.error(
                "[WorkflowGateway] Unmapped workflow status",
                task_id=task_id,
                status=str(description.status),
            )
            raise InternalServerError("Unknown workflow status")

        # Per-step state lives in the workflow and is exposed via a query. This
        # can fail in the brief window before the workflow's first task executes
        # or when no worker is polling -- degrade to an empty step list rather
        # than erroring the whole request.
        try:
            steps = await handle.query("get_step_states", result_type=List[Step])
        except Exception as err:
            self._logger.warn(
                "[WorkflowGateway] Failed to query step states",
                task_id=task_id,
                error=str(err),
            )
            steps = []

        return TaskState(id=task_id, status=status, steps=steps)

    async def list_active_tasks(self) -> Tuple[List[ActiveTask], datetime]:
        """What is running on this robot right now, and when that was read.

        This is the only thing in the system that can answer the question. The
        REST surface has no task collection, the database stores no runs, and a
        schedule-triggered execution's workflow id is never recorded anywhere —
        but every one of them is an execution on this robot's task queue, so
        Temporal's visibility index knows about all of them equally: runs from
        this browser, from another tab, from the MCP server, from a schedule
        that fired while nobody was looking, and runs that started before the
        page asking was ever loaded.

        Cached in one slot for ACTIVE_TASK_CACHE_TTL_S. The console polls this
        from every open tab, and without the cache each of them would spend a
        Temporal RPC on an answer that is identical.
        """
        # Lazily built on first use, on the loop that will own it. Racing to
        # create it is not possible: everything here runs on that single loop.
        if self._active_lock is None:
            self._active_lock = asyncio.Lock()

        cached = self._read_active_cache()
        if cached is not None:
            return cached

        async with self._active_lock:
            # Re-check inside the lock. This is not belt and braces: without it,
            # N callers arriving in the same tick all miss, all queue on the
            # lock, and each then makes its own RPC — N *serialised* round trips,
            # which is worse than no lock at all. With it, one fetches and the
            # rest replay what it stored.
            cached = self._read_active_cache()
            if cached is not None:
                return cached

            snapshot = await self._fetch_active_tasks()
            self._active_snapshot = snapshot

        if snapshot.error is not None:
            raise snapshot.error
        return snapshot.tasks or [], snapshot.as_of

    def _read_active_cache(self) -> Optional[Tuple[List[ActiveTask], datetime]]:
        """Replay the snapshot while it is fresh, re-raising a cached failure.

        Failures are cached in the same slot as successes, and that matters more
        than the success path: a gRPC connect to a dead Temporal can block for
        seconds, and the lock above serialises those attempts, so an uncached
        failure with four tabs polling would pile waiters onto the event loop.
        The cost is that the first success after a recovery is up to one TTL
        late, which is the right trade for a state that changes a few times an
        hour.
        """
        snapshot = self._active_snapshot
        if snapshot is None:
            return None
        if time.monotonic() - snapshot.fetched_at >= ACTIVE_TASK_CACHE_TTL_S:
            return None
        if snapshot.error is not None:
            raise snapshot.error
        return snapshot.tasks or [], snapshot.as_of

    async def _fetch_active_tasks(self) -> _ActiveSnapshot:
        """One visibility query, packaged as a snapshot. Never raises."""
        fetched_at = time.monotonic()
        as_of = datetime.now(timezone.utc)

        def failed(err: Exception) -> _ActiveSnapshot:
            return _ActiveSnapshot(fetched_at=fetched_at, as_of=as_of, error=err)

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            return failed(InternalServerError("Failed to connect to Temporal server"))

        tasks: List[ActiveTask] = []
        try:
            # No `await` on list_workflows: it returns the iterator directly.
            # This is the call the warning in list_schedules was written for —
            # the two APIs differ, and awaiting this one is a TypeError.
            async for execution in client.list_workflows(
                _active_query(self._task_queue),
                limit=ACTIVE_TASK_LIST_LIMIT,
                page_size=ACTIVE_TASK_LIST_LIMIT,
                rpc_timeout=timedelta(seconds=ACTIVE_TASK_RPC_TIMEOUT_S),
            ):
                status = _WORKFLOW_STATUS_MAP.get(execution.status)
                if status is None:
                    # Skipped rather than raised: one unmapped status must not
                    # cost the operator the whole answer, and the query already
                    # filtered on Running so this is a Temporal-side surprise.
                    self._logger.warn(
                        "[WorkflowGateway] Unmapped status in active tasks",
                        task_id=execution.id,
                        status=str(execution.status),
                    )
                    continue

                schedule_id = _schedule_id_of(execution)
                tasks.append(
                    ActiveTask(
                        id=execution.id,
                        run_id=execution.run_id,
                        status=status,
                        started_at=execution.start_time.astimezone(timezone.utc),
                        source=(
                            TaskSource.SCHEDULE if schedule_id else TaskSource.DIRECT
                        ),
                        schedule_id=schedule_id,
                    )
                )
        except RPCError as err:
            if err.status == RPCStatusCode.INVALID_ARGUMENT:
                # Almost certainly a deployment on *standard* visibility, where
                # a List Filter this shape is rejected. The fix is to drop the
                # TaskQueue / WorkflowType predicates from _active_query and
                # filter the results in Python; it is not pre-built because the
                # pinned server (auto-setup 1.29.7 on Postgres) has advanced
                # visibility and one code path is worth more than a fallback
                # nothing exercises.
                self._logger.error(
                    "[WorkflowGateway] Visibility rejected the active-task query; "
                    "is this server on standard visibility?",
                    query=_active_query(self._task_queue),
                    error=str(err),
                )
            else:
                self._logger.error(
                    "[WorkflowGateway] Failed to list active tasks", error=str(err)
                )
            return failed(InternalServerError("List active tasks failed"))
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to list active tasks", error=str(err)
            )
            return failed(InternalServerError("List active tasks failed"))

        # Debug rather than info: this is the cache-miss line, i.e. the one that
        # says how often Temporal is actually being asked. It is the only way to
        # tell coalescing from a cache that is quietly doing nothing.
        self._logger.debug(
            "[WorkflowGateway] Active task snapshot refreshed", count=len(tasks)
        )
        return _ActiveSnapshot(fetched_at=fetched_at, as_of=as_of, tasks=tasks)

    async def cancel_task(self, task_id: str):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_workflow_handle(task_id)

        try:
            await handle.cancel()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Task {task_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to cancel workflow", error=str(err)
            )
            raise InternalServerError("Cancel workflow failed")

    async def create_schedule(self, schedule: ScheduleTask):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        spec = _build_schedule_spec(schedule.trigger)

        # Each scheduled trigger starts a RobotWorkflow. Temporal appends the
        # nominal trigger time to this id, so every run is a distinct execution
        # that can be queried via GET /api/v1/tasks/{id}.
        workflow_task = WorkflowTask(id=schedule.id, definition=schedule.definition)

        # Trigger + provenance into the memo, so get/list can echo both back.
        # See _schedule_to_memo for why the memo and not the action args.
        memo = _schedule_to_memo(schedule)

        try:
            await client.create_schedule(
                schedule.id,
                Schedule(
                    action=ScheduleActionStartWorkflow(
                        WORKFLOW_TYPE_NAME,
                        args=[workflow_task],
                        id=schedule.id,
                        task_queue=self._task_queue,
                    ),
                    spec=spec,
                    # A single robot can only do one thing at a time: never let a
                    # new run start while the previous one is still executing.
                    policy=SchedulePolicy(overlap=ScheduleOverlapPolicy.SKIP),
                ),
                memo=memo or None,
            )
        except ScheduleAlreadyRunningError:
            raise BadRequestError(f"Schedule {schedule.id} already exists")
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to create schedule", error=str(err)
            )
            raise InternalServerError("Create schedule failed")

    async def get_schedule(self, schedule_id: str) -> ScheduleView:

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_schedule_handle(schedule_id)

        try:
            desc = await handle.describe()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Schedule {schedule_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to describe schedule", error=str(err)
            )
            raise InternalServerError("Describe schedule failed")

        memo = await _read_memo(desc)

        return ScheduleView(
            id=desc.id,
            trigger=_trigger_from_memo(memo, desc.schedule.spec),
            paused=desc.schedule.state.paused,
            next_run_times=[
                t.astimezone(timezone.utc) for t in desc.info.next_action_times
            ],
            map_name=memo.get("map_name"),
            saved_task_id=memo.get("saved_task_id"),
            saved_task_name=memo.get("saved_task_name"),
            # Only the describe path can reach the frozen steps; see _read_steps.
            steps=await _read_steps(self._logger, desc),
        )

    async def list_schedules(self) -> List[ScheduleView]:

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        schedules: List[ScheduleView] = []
        try:
            # `list_schedules` is a coroutine returning the iterator, hence the
            # await inside the async-for. (`list_workflows`, by contrast, returns
            # the iterator directly — do not copy this shape onto that one.)
            async for item in await client.list_schedules():
                memo = await _read_memo(item)
                schedules.append(
                    ScheduleView(
                        id=item.id,
                        trigger=_trigger_from_memo(memo, item.schedule.spec),
                        paused=item.schedule.state.paused,
                        next_run_times=[
                            t.astimezone(timezone.utc)
                            for t in item.info.next_action_times
                        ],
                        map_name=memo.get("map_name"),
                        saved_task_id=memo.get("saved_task_id"),
                        saved_task_name=memo.get("saved_task_name"),
                        # `steps` deliberately left empty. A schedule *list*
                        # element is a ScheduleListDescription whose action is a
                        # ScheduleListActionStartWorkflow -- one field, the
                        # workflow type name. The frozen args are unreachable
                        # here at any price, and faking it with a describe() per
                        # row would turn one paged RPC into 1+N on first paint
                        # for data most rows never show. GET /{id} is the path
                        # that carries steps.
                    )
                )
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to list schedules", error=str(err)
            )
            raise InternalServerError("List schedules failed")

        return schedules

    async def delete_schedule(self, schedule_id: str):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_schedule_handle(schedule_id)

        try:
            await handle.delete()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Schedule {schedule_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to delete schedule", error=str(err)
            )
            raise InternalServerError("Delete schedule failed")

    async def pause_schedule(self, schedule_id: str):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_schedule_handle(schedule_id)

        try:
            await handle.pause()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Schedule {schedule_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to pause schedule", error=str(err)
            )
            raise InternalServerError("Pause schedule failed")

    async def resume_schedule(self, schedule_id: str):

        try:
            client = await self._get_client()
        except Exception as err:
            self._logger.error(
                "[WorkflowGateway] Failed to connect to Temporal server", error=str(err)
            )
            raise InternalServerError("Failed to connect to Temporal server")

        handle = client.get_schedule_handle(schedule_id)

        try:
            await handle.unpause()
        except RPCError as err:
            if err.status == RPCStatusCode.NOT_FOUND:
                raise NotFoundError(f"Schedule {schedule_id} not found")
            self._logger.error(
                "[WorkflowGateway] Failed to resume schedule", error=str(err)
            )
            raise InternalServerError("Resume schedule failed")


def init_workflow_gateway(
    logger: structlog.stdlib.BoundLogger,
    robot_id: str,
) -> WorkflowGateway:
    workflow_gw = WorkflowGateway(logger=logger, robot_id=robot_id)
    return workflow_gw
