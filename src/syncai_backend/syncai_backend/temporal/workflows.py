import asyncio
from datetime import timedelta

from temporalio import workflow
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
            StepType.ARTIFACT: RobotActivities.execute_artifact,
        }

        for step in self._steps:
            activity_fn = activity_map.get(step.type)
            if activity_fn is None:
                step.status = StepStatus.FAILED
                step.error_msg = f"Unknown step type: {step.type}"
                raise ApplicationError(
                    f"Unknown step type: {step.type}", non_retryable=True
                )

            step.status = StepStatus.IN_PROGRESS
            try:
                result: ActivityResult = await workflow.execute_activity(
                    activity_fn,
                    step.params,
                    start_to_close_timeout=timedelta(minutes=3600),
                    heartbeat_timeout=timedelta(seconds=3),
                    cancellation_type=workflow.ActivityCancellationType.WAIT_CANCELLATION_COMPLETED,
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
