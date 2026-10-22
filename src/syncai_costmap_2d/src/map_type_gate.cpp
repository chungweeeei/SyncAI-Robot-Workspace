#include "syncai_costmap_2d/map_type_gate.hpp"

#include <stdexcept>
#include <string>

namespace syncai_costmap_2d
{

void MapTypeGate::initialize(
  const rclcpp::Node::SharedPtr & node, const std::string & layer_name,
  const std::string & map_type, const std::string & map_3d_topic,
  const rclcpp::CallbackGroup::SharedPtr & callback_group)
{
  if (map_type == "any") {
    want_ = Want::Any;
  } else if (map_type == "2d") {
    want_ = Want::Map2D;
  } else if (map_type == "3d") {
    want_ = Want::Map3D;
  } else {
    throw std::runtime_error(
      layer_name + ".map_type must be \"any\", \"2d\" or \"3d\", not \"" + map_type + "\"");
  }
  last_active_ = active();
  first_ = true;
  if (want_ == Want::Any) {
    return;
  }

  auto logger = node->get_logger();
  rclcpp::SubscriptionOptions opt;
  opt.callback_group = callback_group;
  // Latched on the planner's side (reliable + transient_local), so the
  // current value arrives on subscribe, whenever the planner came up.
  sub_ = node->create_subscription<std_msgs::msg::Bool>(
    map_3d_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this, logger, layer_name, map_3d_topic](const std_msgs::msg::Bool::SharedPtr msg) {
      if (map_3d_.exchange(msg->data) != msg->data) {
        RCLCPP_INFO(
          logger, "%s: %s map (%s) -- layer %s", layer_name.c_str(), msg->data ? "3D" : "2D",
          map_3d_topic.c_str(), active() ? "on" : "off");
      }
    },
    opt);
  // The map reads 2D until the first message, as the task_runner's does.
  RCLCPP_INFO(
    logger, "%s runs on %s maps only (%s); %s until the planner says otherwise", layer_name.c_str(),
    want_ == Want::Map3D ? "3D" : "2D", map_3d_topic.c_str(), active() ? "on" : "off");
}

bool MapTypeGate::active() const
{
  switch (want_) {
    case Want::Map2D:
      return !map_3d_.load();
    case Want::Map3D:
      return map_3d_.load();
    default:
      return true;
  }
}

bool MapTypeGate::changed()
{
  const bool now = active();
  const bool out = first_ || now != last_active_;
  first_ = false;
  last_active_ = now;
  return out;
}

}  // namespace syncai_costmap_2d
