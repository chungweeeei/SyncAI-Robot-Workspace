from datetime import timedelta

from temporalio import workflow
from temporalio.exceptions import ApplicationError

with workflow.unsafe.imports_passed_through():
    from syncai_backend.gateways.workflow.schema import WorkflowTask, StepType
    from syncai_backend.temporal.activities import ActivityResult, RobotActivities


@workflow.defn
class RobotWorkflow:
    @workflow.run
    async def run(self, task: WorkflowTask):
        activity_map = {
            StepType.MOVE: RobotActivities.execute_move,
        }

        for step in task.definition.steps:
            activity_fn = activity_map.get(step.type)
            if activity_fn is None:
                raise ApplicationError(
                    f"Unknown step type: {step.type}", non_retryable=True
                )

            result: ActivityResult = await workflow.execute_activity(
                activity_fn,
                step.params,
                start_to_close_timeout=timedelta(minutes=60),
                heartbeat_timeout=timedelta(seconds=10),
            )

            if not result.success:
                raise ApplicationError("activity failed", non_retryable=True)

        return
