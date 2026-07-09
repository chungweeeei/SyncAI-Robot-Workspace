#include "syncai_map_server/costmap_filter_info_server.hpp"

#include <memory>
#include <string>
#include <utility>

namespace syncai_map_server
{

CostmapFilterInfoServer::CostmapFilterInfoServer(const rclcpp::NodeOptions & options)
: Node("costmap_filter_info_server", options)
{
  this->declare_parameter("filter_info_topic", "costmap_filter_info");
  this->declare_parameter("type", 0);
  this->declare_parameter("mask_topic", "filter_mask");
  this->declare_parameter("base", 0.0);
  this->declare_parameter("multiplier", 1.0);

  const std::string filter_info_topic = this->get_parameter("filter_info_topic").as_string();

  // Latched (transient_local) so late-joining costmap filters receive the info
  // immediately, same QoS as nav2's CostmapFilterInfoServer.
  publisher_ = this->create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto msg = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
  msg->header.frame_id = "";
  msg->header.stamp = now();
  msg->type = this->get_parameter("type").as_int();
  msg->filter_mask_topic = this->get_parameter("mask_topic").as_string();
  msg->base = static_cast<float>(this->get_parameter("base").as_double());
  msg->multiplier = static_cast<float>(this->get_parameter("multiplier").as_double());

  RCLCPP_INFO(
    this->get_logger(),
    "[CostmapFilterInfoServer][%s] Publishing filter info (type: %d, mask_topic: %s) on %s",
    __func__, msg->type, msg->filter_mask_topic.c_str(), filter_info_topic.c_str());

  publisher_->publish(std::move(msg));
}

CostmapFilterInfoServer::~CostmapFilterInfoServer()
{
}

}  // namespace syncai_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(syncai_map_server::CostmapFilterInfoServer)
