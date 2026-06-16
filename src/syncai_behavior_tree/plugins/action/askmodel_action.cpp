#include "syncai_behavior_tree/plugins/action/askmodel_action.hpp"

#include <memory>
#include <string>

namespace syncai_behavior_tree
{
AskmodelAction::AskmodelAction(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<syncai_test_bt_action_msgs::action::Askmodel>(xml_tag_name, action_name, conf)
{
}

void AskmodelAction::on_tick()
{
  getInput("question", goal_.question);
}

void AskmodelAction::on_wait_for_result(
  std::shared_ptr<const syncai_test_bt_action_msgs::action::Askmodel::Feedback> feedback)
{
  if (feedback) {
    RCLCPP_INFO(node_->get_logger(), "[Askmodel] received feedback");
  }
}

BT::NodeStatus AskmodelAction::on_success()
{
  setOutput("success", result_.result->success);
  setOutput("answer", result_.result->answer);
  RCLCPP_INFO(node_->get_logger(), "[Askmodel] succeeded: %s", result_.result->answer.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AskmodelAction::on_aborted()
{
  setOutput("success", false);
  setOutput("answer", std::string("aborted"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus AskmodelAction::on_cancelled()
{
  setOutput("success", false);
  setOutput("answer", std::string("cancelled"));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace syncai_behavior_tree

// 讓這個 .so 被 BehaviorTreeEngine 載入時，自動向 factory 註冊 "AskModel" 這個 XML tag。
// 注意：BT_REGISTER_NODES 必須在 global scope（namespace 之外），否則巨集展開的
// extern "C" 函式內 BT:: 會被當成 syncai_behavior_tree::BT:: 而找不到型別。
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // builder 決定「遇到 XML tag 時怎麼建構這個 node」——第二個參數 "askmodel"
  // 就是預設要連線的 action 名稱（對到 test_action_server 的 "/askmodel"）。
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::AskmodelAction>(name, "askmodel", config);
  };

  factory.registerBuilder<syncai_behavior_tree::AskmodelAction>("AskModel", builder);
}