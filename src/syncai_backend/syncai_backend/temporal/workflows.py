from datetime import timedelta

from temporalio import workflow
from temporalio.exceptions import ApplicationError

with workflow.unsafe.imports_passed_through():
    from gateways.workflow.schema import WorkflowTask, StepResult, StepType
    from temporal.activities import TaskActivities


@workflow.defn
class TaskWorkflow:
    @workflow.run
    async def run(self, task: WorkflowTask) -> StepResult:
        activity_map = {
            StepType.MOVE: TaskActivities.execute_move,
        }

        for step in task.definition.steps:
            activity_fn = activity_map.get(step.type)
            if activity_fn is None:
                raise ApplicationError(
                    f"Unknown step type: {step.type}", non_retryable=True
                )

            result: StepResult = await workflow.execute_activity(
                activity_fn,
                step,
                start_to_close_timeout=timedelta(minutes=60),
            )

            if not result.success:
                raise ApplicationError(result.message, non_retryable=True)

        return StepResult(success=True, message="Task completed")
