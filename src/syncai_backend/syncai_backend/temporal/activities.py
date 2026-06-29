import structlog

from temporalio import activity

from gateways.workflow.schema import StepRequest, StepResult


class TaskActivities:
    """Robot task activities executed by the Temporal worker.

    建構子目前只注入 logger；之後接 robot gateway 時再從這裡注入並於各
    activity 內呼叫真正的硬體 / ROS 動作。
    """

    def __init__(self, logger: structlog.stdlib.BoundLogger):
        self._logger = logger

    @activity.defn
    async def execute_move(self, step: StepRequest) -> StepResult:
        self._logger.info(
            "[Activity] execute_move",
            step_id=step.id,
            params=step.params.model_dump(),
        )
        # TODO: 後續接 robot gateway 實作真正的導航。
        return StepResult(success=True, message="MOVE completed")
