#include "syncai_behavior_tree/plugins/action/printout_service.hpp"

#include <memory>
#include <string>

namespace syncai_behavior_tree
{
PrintoutService::PrintoutService(
  const std::string & service_node_name, const BT::NodeConfiguration & conf,
  const std::string & service_name)
: BtServiceNode<syncai_test_bt_action_msgs::srv::Printout>(service_node_name, conf, service_name)
{
}

void PrintoutService::on_tick()
{
  getInput("text", request_->text);
}

BT::NodeStatus PrintoutService::on_completion(
  std::shared_ptr<syncai_test_bt_action_msgs::srv::Printout::Response> response)
{
  setOutput("success", response->success);

  if (response->success) {
    RCLCPP_INFO(node_->get_logger(), "[Printout] service succeeded");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(node_->get_logger(), "[Printout] service reported failure");
  return BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

// 讓這個 .so 被 BehaviorTreeEngine 載入時，自動向 factory 註冊 "Printout" 這個 XML tag。
// 注意：BT_REGISTER_NODES 必須在 global scope（namespace 之外），否則巨集展開的
// extern "C" 函式內 BT:: 會被當成 syncai_behavior_tree::BT:: 而找不到型別。
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // builder 決定「遇到 XML tag 時怎麼建構這個 node」——第三個參數 "printout"
  // 就是預設要連線的 service 名稱。
  BT::NodeBuilder builder = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::PrintoutService>(name, config, "printout");
  };

  factory.registerBuilder<syncai_behavior_tree::PrintoutService>("Printout", builder);
}
