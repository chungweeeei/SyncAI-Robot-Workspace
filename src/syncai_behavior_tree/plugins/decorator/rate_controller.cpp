#include "syncai_behavior_tree/plugins/decorator/rate_controller.hpp"

#include <chrono>
#include <string>

namespace syncai_behavior_tree
{

RateController::RateController(const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf), first_time_(false)
{
  double hz = 1.0;
  getInput("hz", hz);
  period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick()
{
  // When the BT re-enters this node, reset the timing origin
  if (status() == BT::NodeStatus::IDLE) {
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  // Set this node's status to RUNNING
  setStatus(BT::NodeStatus::RUNNING);

  // Compute how much time has elapsed since start
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // Decide whether to tick the child; any one of the following conditions is enough:
  // - first_time_: this is the first round
  // - the child's current status is RUNNING
  // - at least period_ has elapsed since the last tick

  if (
    first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_) {
    first_time_ = false;
    // Tick the child here and hand its result back to the parent
    const BT::NodeStatus child_state = child_node_->executeTick();

    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }

  return status();
}
}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::RateController>("RateController");
}
