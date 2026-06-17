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
  // 當 BT 重新進入這個 node 時，reset 計時起點
  if (status() == BT::NodeStatus::IDLE) {
    start_ = std::chrono::high_resolution_clock::now();
    first_time_ = true;
  }

  // 把自己的 node status 設成 running
  setStatus(BT::NodeStatus::RUNNING);

  // 計算距離 start 已經過了多久
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = now - start_;

  typedef std::chrono::duration<float> float_seconds;
  auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

  // 判斷要不要 tick child node，只要符合以下任一條件就 tick
  // - first_time_： 這一輪是第一次
  // - child node 目前 status 是 RUNNING
  // - 距離上次 tick 已經過了 period_ 這麼久

  if (
    first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
    seconds.count() >= period_) {
    first_time_ = false;
    // 這裡去 tick child node，並把結果回傳給 parent node
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
