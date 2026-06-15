#include <memory>
#include <string>

#include "syncai_behavior_tree/plugins/action/countdown_action.hpp"

namespace syncai_behavior_tree
{

CountdownAction::CountdownAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<syncai_test_action::action::Countdown>(xml_tag_name, action_name, conf)
{
}

void CountdownAction::on_tick()
{
  // 從 XML port "seconds" 讀值，填進要送出的 goal_。
  getInput("seconds", goal_.seconds);
}

void CountdownAction::on_wait_for_result(
  std::shared_ptr<const syncai_test_action::action::Countdown::Feedback> feedback)
{
  // feedback 在沒有新訊息時會是 nullptr（基底在每個 tick 後會 reset）。
  if (feedback) {
    RCLCPP_INFO(
      node_->get_logger(), "[Countdown] remaining: %d", feedback->remaining_seconds);
  }
}

BT::NodeStatus CountdownAction::on_success()
{
  setOutput("success", result_.result->success);
  setOutput("message", result_.result->message);
  RCLCPP_INFO(
    node_->get_logger(), "[Countdown] succeeded: %s", result_.result->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CountdownAction::on_aborted()
{
  setOutput("success", false);
  setOutput("message", std::string("aborted"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CountdownAction::on_cancelled()
{
  setOutput("success", false);
  setOutput("message", std::string("cancelled"));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace syncai_behavior_tree

// 讓這個 .so 被 BehaviorTreeEngine 載入時，自動向 factory 註冊 "Countdown" 這個 XML tag。
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // builder 決定「遇到 XML tag 時怎麼建構這個 node」——第二個參數 "countdown"
  // 就是預設要連線的 action 名稱（對到 test_action_server 的 "/countdown"）。
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::CountdownAction>(name, "countdown", config);
  };

  factory.registerBuilder<syncai_behavior_tree::CountdownAction>("Countdown", builder);
}
