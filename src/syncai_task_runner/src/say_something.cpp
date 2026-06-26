#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace syncai_behavior_tree
{
class SaySomethingService
{
public:
  bool on_initialize(rclcpp::Node::WeakPtr parent, const std::string & service_name)
  {
    auto node = parent.lock();
    if (!node) {
      return false;
    }

    say_something_service_ = node->create_service<std_srvs::srv::Trigger>(
      service_name,
      std::bind(
        &SaySomethingService::handle_request, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      node->get_logger(), "[SaySomethingService] Service '%s' initialized.", service_name.c_str());
    return true;
  }

private:
  void handle_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = true;
    response->message = "done";
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr say_something_service_;
};
}  // namespace syncai_behavior_tree