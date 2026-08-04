import structlog
from datetime import timedelta, timezone
from typing import List

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
from temporalio.service import RPCError, RPCStatusCode

from syncai_backend.exceptions import (
    BadRequestError,
    InternalServerError,
    NotFoundError,
)

from syncai_backend.temporal.shared import TEMPORAL_SERVER_URL

from syncai_backend.gateways.workflow.schema import (
    ScheduleTask,
    ScheduleTrigger,
    ScheduleView,
    Step,
    TaskState,
    WorkflowTask,
)
from syncai_backend.gateways.workflow.config import WORKFLOW_TYPE_NAME


_WORKFLOW_STATUS_MAP = {
    WorkflowExecutionStatus.RUNNING: "IN_PROGRESS",
    WorkflowExecutionStatus.CONTINUED_AS_NEW: "IN_PROGRESS",
    WorkflowExecutionStatus.COMPLETED: "COMPLETED",
    WorkflowExecutionStatus.FAILED: "FAILED",
    WorkflowExecutionStatus.TIMED_OUT: "FAILED",
    WorkflowExecutionStatus.CANCELED: "CANCELED",
    WorkflowExecutionStatus.TERMINATED: "CANCELED",
}


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
