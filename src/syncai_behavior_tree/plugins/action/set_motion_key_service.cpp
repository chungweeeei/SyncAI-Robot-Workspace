#include "syncai_behavior_tree/plugins/action/set_motion_key_service.hpp"

#include <memory>
#include <string>

namespace syncai_behavior_tree
{

SetMotionKeyService::SetMotionKeyService(
  const std::string & service_node_name, const BT::NodeConfiguration & conf)
// The third argument is the service name used when the XML sets no
// service_name port (providedBasicPorts() declares that port with a
// description, not a default, so the port only overrides).
: BtServiceNode<syncai_common::srv::SetMotionKey>(service_node_name, conf, "set_motion_key")
{
}

void SetMotionKeyService::on_tick()
{
  if (!getInput("key", request_->key) || request_->key.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(), "[SetMotionKeyService][%s] the key port is not set; not calling %s",
      __func__, service_name_.c_str());
    report_failure("SetMotionKey: the key port is not set");
    should_send_request_ = false;
    return;
  }
  key_ = request_->key;
  RCLCPP_INFO(
    node_->get_logger(), "[SetMotionKeyService][%s] sending motion key '%s' to %s", __func__,
    key_.c_str(), service_name_.c_str());
}

BT::NodeStatus SetMotionKeyService::on_completion(
  std::shared_ptr<syncai_common::srv::SetMotionKey::Response> response)
{
  // `success` false is the driver refusing: "LOCKED" while its safety lock is
  // engaged (everything but the e-stop key), or an unknown key. True means
  // the datagram left; whether the gait changed is not knowable here.
  if (!response->success) {
    RCLCPP_ERROR(
      node_->get_logger(), "[SetMotionKeyService][%s] %s refused key '%s': %s", __func__,
      service_name_.c_str(), key_.c_str(), response->message.c_str());
    report_failure(service_name_ + " '" + key_ + "': " + response->message);
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(
    node_->get_logger(), "[SetMotionKeyService][%s] key '%s' sent: %s", __func__, key_.c_str(),
    response->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

void SetMotionKeyService::on_timeout()
{
  // Unattributed, the same rule as BtActionNode's unacknowledged goal: a
  // silent driver says the stack is not all there, not that docking failed.
  report_failure(
    service_name_ + " '" + key_ + "' did not answer within " +
      std::to_string(server_timeout_.count()) + " ms",
    /*attributed=*/false);
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::SetMotionKeyService>("SetMotionKey");
}
