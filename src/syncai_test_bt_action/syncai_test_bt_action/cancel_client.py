"""Test client for the /ask_model action.

Sends a goal, waits a configurable delay, then cancels it -- useful for
verifying that the server honors cancellation while a model call is running.

Usage:
    ros2 run syncai_test_bt_action cancel_client
    ros2 run syncai_test_bt_action cancel_client --delay 2.0
    ros2 run syncai_test_bt_action cancel_client --no-cancel   # just run to completion
    ros2 run syncai_test_bt_action cancel_client --question "your prompt here"
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from syncai_test_bt_action_msgs.action import Askmodel

# Status codes from action_msgs/msg/GoalStatus
_STATUS = {2: "EXECUTING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


class CancelClient(Node):
    def __init__(self, question: str, delay: float, do_cancel: bool):
        super().__init__("cancel_client")
        self._question = question
        self._delay = delay
        self._do_cancel = do_cancel
        self._client = ActionClient(self, Askmodel, "ask_model")

    def _on_feedback(self, msg):
        self.get_logger().info(f"Feedback: state={msg.feedback.state}")

    def run(self) -> int:
        self.get_logger().info("Waiting for action server...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server /ask_model not available.")
            return 1

        goal = Askmodel.Goal()
        goal.question = self._question
        self.get_logger().info(f"Sending goal: {self._question!r}")
        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return 1
        self.get_logger().info("Goal accepted.")

        if self._do_cancel:
            # Keep the node spinning (so feedback arrives) during the delay.
            t0 = time.time()
            while time.time() - t0 < self._delay:
                rclpy.spin_once(self, timeout_sec=0.05)

            self.get_logger().info(f"Requesting cancel after {self._delay}s...")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            if cancel_future.result().goals_canceling:
                self.get_logger().info("Server accepted the cancel request.")
            else:
                self.get_logger().warn("Server did NOT accept the cancel.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapper = result_future.result()
        status = _STATUS.get(wrapper.status, str(wrapper.status))
        self.get_logger().info(
            f"Final status: {status} | "
            f"success={wrapper.result.success} answer={wrapper.result.answer!r}"
        )
        return 0


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--question",
        default="Who are you?",
    )
    parser.add_argument(
        "--delay", type=float, default=1.5, help="seconds to wait before cancelling"
    )
    parser.add_argument(
        "--no-cancel",
        dest="cancel",
        action="store_false",
        help="let the goal run to completion instead",
    )
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = CancelClient(args.question, args.delay, args.cancel)
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
