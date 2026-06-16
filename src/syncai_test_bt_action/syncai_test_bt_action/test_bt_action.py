import os
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import dotenv
from google import genai
from google.genai import types

from syncai_test_bt_action_msgs.srv import Printout
from syncai_test_bt_action_msgs.action import Askmodel


dotenv.load_dotenv()


class TestBtActionNode(Node):
    def __init__(self):
        super().__init__("test_bt_action")
        self.get_logger().info("test_bt_action node has started.")

        # Gemini client. Read the API key explicitly from the environment
        # (dotenv.load_dotenv() above loads it from a .env file if present).
        api_key = os.environ.get("GEMINI_API_KEY")
        if not api_key:
            raise RuntimeError(
                "GEMINI_API_KEY is not set. Export it or put it in a .env file."
            )
        self._model = "gemini-2.5-flash"
        self._client = genai.Client(api_key=api_key)

        # Disable "thinking" so the model starts streaming output immediately
        # instead of pausing several seconds for internal reasoning first.
        self._gen_config = types.GenerateContentConfig(
            thinking_config=types.ThinkingConfig(thinking_budget=0)
        )

        # Thread pool to run the streaming call off the executor thread, so
        # _execute only polls and reacts to cancellation immediately.
        self._pool = ThreadPoolExecutor(max_workers=2)

        # register action server.
        # ReentrantCallbackGroup so the cancel service can run concurrently
        # with _execute (a MutuallyExclusiveCallbackGroup would serialize them
        # and is_cancel_requested would never flip during execution).
        self._action_server = ActionServer(
            node=self,
            action_type=Askmodel,
            action_name="ask_model",
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self._handle_goal,
            handle_accepted_callback=self._handle_accepted,
            execute_callback=self._execute,
            cancel_callback=self._handle_cancel,
        )

        # register service
        self._print_srv = self.create_service(
            srv_type=Printout,
            srv_name="printout",
            callback=self._handle_printout,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _handle_goal(self, goal_request: Askmodel.Goal) -> GoalResponse:
        # goal_callback receives the Goal message, not a handle.
        self.get_logger().info(
            f"Received goal request: question={goal_request.question!r}"
        )
        return GoalResponse.ACCEPT

    def _handle_accepted(self, goal_handle: ServerGoalHandle) -> None:
        self.get_logger().info("Goal accepted, starting execution.")
        goal_handle.execute()

    def _execute(self, goal_handle: ServerGoalHandle) -> Askmodel.Result:
        question = goal_handle.request.question
        self.get_logger().info(f"Executing goal: {question!r}")

        feedback = Askmodel.Feedback()
        result = Askmodel.Result()
        feedback.state = Askmodel.Feedback.THINKING

        # Run the streaming call in a worker thread. _execute itself only
        # polls, so it reacts to cancellation immediately instead of blocking
        # on the next chunk; the worker closes the stream when it sees the
        # cancel flag, tearing down the HTTP connection early.
        future = self._pool.submit(self._stream_model, question, goal_handle)

        while not future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal canceled.")
                goal_handle.canceled()
                result.success = False
                result.answer = ""
                # Don't wait for the worker; it will close the stream and
                # exit on its own at the next chunk boundary.
                return result
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        try:
            answer = future.result()
        except Exception as exc:  # network / quota / auth errors
            self.get_logger().error(f"Gemini call failed: {exc}")
            result.success = False
            result.answer = ""
            goal_handle.abort()
            return result

        feedback.state = Askmodel.Feedback.FINISHED
        goal_handle.publish_feedback(feedback)

        result.success = True
        result.answer = answer
        goal_handle.succeed()
        self.get_logger().info(f"Goal succeeded: {answer!r}")
        return result

    def _stream_model(self, question: str, goal_handle: ServerGoalHandle) -> str:
        """Run in a worker thread: stream the model response, stopping early
        (closing the connection) if cancellation is requested."""
        chunks = []
        stream = self._client.models.generate_content_stream(
            model=self._model,
            contents=question,
            config=self._gen_config,
        )
        for chunk in stream:
            if goal_handle.is_cancel_requested:
                self._close_stream(stream)
                break
            if chunk.text:
                chunks.append(chunk.text)
        return "".join(chunks)

    @staticmethod
    def _close_stream(stream) -> None:
        # The streaming response is a generator; closing it raises
        # GeneratorExit inside and releases the underlying HTTP connection.
        closer = getattr(stream, "close", None)
        if callable(closer):
            closer()

    def _handle_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    def _handle_printout(
        self, request: Printout.Request, response: Printout.Response
    ) -> Printout.Response:
        self.get_logger().info(
            f"Printout service called with message: {request.text!r}"
        )
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TestBtActionNode()
    # MultiThreadedExecutor so other callbacks aren't starved while a goal
    # is executing.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
