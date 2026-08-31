import asyncio
from datetime import timedelta

from temporalio import workflow
from temporalio.common import RetryPolicy
from temporalio.exceptions import ActivityError, ApplicationError

with workflow.unsafe.imports_passed_through():
    from syncai_backend.gateways.workflow.schema import (
        Step,
        StepStatus,
        StepType,
        WorkflowTask,
    )
    from syncai_backend.temporal.activities import ActivityResult, RobotActivities


@workflow.defn
class RobotWorkflow:
    def __init__(self) -> None:
        self._steps: list[Step] = []

    @workflow.query
    def get_step_states(self) -> list[Step]:
        return self._steps

    @workflow.run
    async def run(self, task: WorkflowTask):
        self._steps = task.definition.steps

        activity_map = {
            StepType.MOVE: RobotActivities.execute_move,
            StepType.STANDUP: RobotActivities.execute_stand,
            StepType.LIEDOWN: RobotActivities.execute_lie_down,
            StepType.SPEAK: RobotActivities.execute_speak,
        }

        for step in self._steps:
            activity_fn = activity_map.get(step.type)
            if activity_fn is None:
                step.status = StepStatus.FAILED
                step.error_msg = f"Unknown step type: {step.type}"
                raise ApplicationError(
                    f"Unknown step type: {step.type}", non_retryable=True
                )

            # The posture activities (STANDUP/LIEDOWN) take no argument, so
            # they must be invoked with an empty arg list -- handing them a
            # None would fail the worker's argument-count check. The schema
            # guarantees params is None for exactly those step types.
            args = [] if step.params is None else [step.params]

            # SPEAK cannot heartbeat: execute_speak sits in a single blocking
            # gateway call (synthesis + aplay for the whole utterance), so the
            # 3s heartbeat_timeout below would kill every attempt before its
            # first heartbeat could ever arrive. Dead-worker detection for
            # SPEAK therefore falls to start_to_close alone -- which is also
            # why it gets a much shorter one than the heartbeating activities:
            # a worst-case utterance (1000 chars at 0.5x speed, plus the
            # one-time model load) is minutes, and an hour of a dead worker
            # silently holding the step would defeat the point of the timeout.
            if step.type is StepType.SPEAK:
                start_to_close_timeout = timedelta(minutes=5)
                heartbeat_timeout = None
            else:
                start_to_close_timeout = timedelta(hours=1)
                heartbeat_timeout = timedelta(seconds=3)

            step.status = StepStatus.IN_PROGRESS
            try:
                result: ActivityResult = await workflow.execute_activity(
                    activity_fn,
                    args=args,
                    # Without an explicit policy Temporal retries forever
                    # (maximum_attempts=0). The activities mark aborted and
                    # rejected moves retryable because those are sometimes
                    # transient -- but against unlimited attempts, a
                    # permanently blocked MOVE re-dispatched every backoff
                    # interval kept this run open forever: the step showed
                    # IN_PROGRESS for good, and ScheduleOverlapPolicy.SKIP
                    # silently dropped every later trigger of the schedule.
                    # Three attempts keeps the self-healing for the transient
                    # cases and turns the persistent ones into a visible
                    # FAILED step. The 5s initial interval is so a MOVE's
                    # second try isn't 1s after the first (the default) --
                    # too soon for e.g. a restarting action server to be back.
                    retry_policy=RetryPolicy(
                        initial_interval=timedelta(seconds=5),
                        maximum_attempts=3,
                    ),
                    # Per-attempt ceiling. This used to be minutes=3600 -- 60
                    # hours, a units slip (the intent was one hour), so it
                    # could never fire. A dead worker is caught by the 3s
                    # heartbeat regardless; this bounds the live-but-stuck
                    # case where an attempt heartbeats forever without ever
                    # reaching a terminal state. (Both values are picked per
                    # step type above -- SPEAK cannot heartbeat.)
                    start_to_close_timeout=start_to_close_timeout,
                    heartbeat_timeout=heartbeat_timeout,
                    cancellation_type=(
                        workflow.ActivityCancellationType.WAIT_CANCELLATION_COMPLETED
                    ),
                )
            except asyncio.CancelledError:
                step.status = StepStatus.CANCELED
                step.error_msg = "Task canceled"
                raise
            except ActivityError as err:
                step.status = StepStatus.FAILED
                step.error_msg = str(err.cause or err)
                raise

            if not result.success:
                step.status = StepStatus.FAILED
                step.error_msg = "activity failed"
                raise ApplicationError("activity failed", non_retryable=True)

            step.status = StepStatus.COMPLETED

        return
