#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_MOTION_STATE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_MOTION_STATE_HPP_

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief Return RUNNING until the gait controller REPORTS a motion state,
 * then SUCCESS; FAILURE after `timeout` ms. The check that the docking trees'
 * fixed Waits after a SetMotionKey stood in for (2026-10-14).
 *
 * Why it is needed: set_motion_key is one-way UDP and its `success` only
 * means the datagram left the driver. The controller's own view comes back
 * on syncai_driver_manager's `mode` topic (std_msgs/Int32MultiArray out of the
 * MODE_STATE telemetry section: data[0] the RL policy, data[1] the motion
 * state -- see syncai_common/msg/RobotLowLevelMode.msg). MPC (key "5",
 * `MODE M`) reports 5 on data[1], read off the robot on 2026-10-14.
 *
 * The LATEST sample counts, whenever it arrived: the subscription lives as
 * long as the tree, so a robot that is already in the state passes at once.
 * Not "a sample newer than onStart()": `mode` is published only when a
 * telemetry datagram carries MODE_STATE, with no fixed rate and no latch, and
 * a freshness rule would turn a quiet link into a timeout for a robot that is
 * in the right state. The cost is the opposite case -- a stale sample from
 * before a switch that silently failed -- which the step after this node
 * (a docking approach the gait does not execute) surfaces as a dock failure
 * rather than a hang.
 *
 * Own callback group and executor, like IsPathValidCondition: spinning the
 * shared task_runner node from inside a tick would run unrelated callbacks.
 * The executor is spun without blocking on every tick, so the node never
 * holds the engine's 50 ms loop.
 *
 * On success it logs how long the transition took -- the measurement the
 * fixed Waits were missing. On timeout it writes failed_node / failure_msg
 * (the report_failure() contract), which the navigator maps to DOCK_FAILED.
 */
class WaitForMotionState : public BT::StatefulActionNode
{
public:
  WaitForMotionState(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("state", "Motion state to wait for (5 = MPC; data[`index`] of `mode`)"),
      BT::InputPort<int>("index", 1, "Index into the mode array: 1 = motion state, 0 = policy"),
      BT::InputPort<int>("timeout", 5000, "Milliseconds before giving up with FAILURE"),
      BT::InputPort<std::string>("topic", "mode", "std_msgs/Int32MultiArray topic, relative"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Drain the subscription and compare the latest sample with the target.
  bool reached();
  void fail(const std::string & msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;

  // The latest sample's data, empty until one has been received. Indexed in
  // reached(), not in the callback, because `index` is a port read per run.
  std::vector<int32_t> latest_;
  int index_{1};
  int target_{0};
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__WAIT_FOR_MOTION_STATE_HPP_
