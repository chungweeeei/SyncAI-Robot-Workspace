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


def _trigger_to_memo(trigger: ScheduleTrigger) -> dict:
    """Serialise a trigger into memo fields so it survives Temporal's cron
    normalisation and can be echoed back verbatim on get/list."""
    memo: dict = {}
    if trigger.cron:
        memo["cron"] = trigger.cron
    if trigger.interval_seconds:
        memo["interval_seconds"] = trigger.interval_seconds
    if trigger.timezone:
        memo["timezone"] = trigger.timezone
    return memo


async def _read_trigger(described) -> ScheduleTrigger:
    """Rebuild a ScheduleTrigger, preferring the original values saved in the
    schedule memo and falling back to the (normalised) spec."""
    try:
        memo = await described.memo()
    except Exception:
        memo = {}

    cron = memo.get("cron") if memo else None
    interval_seconds = memo.get("interval_seconds") if memo else None
    timezone = memo.get("timezone") if memo else None

    if cron or interval_seconds:
        return ScheduleTrigger(
            cron=cron, interval_seconds=interval_seconds, timezone=timezone
        )

    return _spec_to_trigger(described.schedule.spec)


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

        # Temporal normalises cron_expressions into internal calendar specs, so
        # describe() no longer returns the original cron string. Stash the
        # original trigger in the schedule memo so get/list can echo it back.
        memo = _trigger_to_memo(schedule.trigger)

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

        return ScheduleView(
            id=desc.id,
            trigger=await _read_trigger(desc),
            paused=desc.schedule.state.paused,
            next_run_times=[
                t.astimezone(timezone.utc) for t in desc.info.next_action_times
            ],
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
            async for item in await client.list_schedules():
                schedules.append(
                    ScheduleView(
                        id=item.id,
                        trigger=await _read_trigger(item),
                        paused=item.schedule.state.paused,
                        next_run_times=[
                            t.astimezone(timezone.utc)
                            for t in item.info.next_action_times
                        ],
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
