#include "syncai_costmap_2d/layer.hpp"

#include <string>
#include <vector>

#include "syncai_util/node_utils.hpp"

namespace syncai_costmap_2d
{

Layer::Layer() : layered_costmap_(nullptr), name_(), tf_(nullptr), current_(false), enabled_(false)
{
}

void Layer::initialize(
  LayeredCostmap * parent, std::string name, tf2_ros::Buffer * tf,
  const rclcpp::Node::SharedPtr & node, rclcpp::CallbackGroup::SharedPtr callback_group)
{
  layered_costmap_ = parent;
  name_ = name;
  tf_ = tf;
  node_ = node;
  callback_group_ = callback_group;
  {
    logger_ = node_->get_logger();
    clock_ = node_->get_clock();
  }

  onInitialize();
}

const std::vector<geometry_msgs::msg::Point> & Layer::getFootprint() const
{
  return layered_costmap_->getFootprint();
}

void Layer::declareParameter(const std::string & param_name, const rclcpp::ParameterValue & value)
{
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  syncai_util::declare_parameter_if_not_declared(node_, getFullName(param_name), value);
}

void Layer::declareParameter(
  const std::string & param_name, const rclcpp::ParameterType & param_type)
{
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }
  local_params_.insert(param_name);
  syncai_util::declare_parameter_if_not_declared(node_, getFullName(param_name), param_type);
}

bool Layer::hasParameter(const std::string & param_name)
{
  if (!node_) {
    throw std::runtime_error{"Failed to lock node"};
  }
  return node_->has_parameter(getFullName(param_name));
}

std::string Layer::getFullName(const std::string & param_name)
{
  return std::string(name_ + "." + param_name);
}

std::string Layer::joinWithParentNamespace(const std::string & topic)
{
  if (!node_) {
    throw std::runtime_error{"Failed to get node in Layer::joinWithParentNamespace"};
  }

  // Absolute topics are used as-is.
  if (!topic.empty() && topic.front() == '/') {
    return topic;
  }

  // The costmap node namespace is e.g. "/robot01/global_costmap"; strip the last
  // segment (the costmap name) to get the parent namespace "/robot01", then join.
  std::string node_namespace = node_->get_namespace();
  std::string parent_namespace = node_namespace.substr(0, node_namespace.find_last_of('/'));
  return parent_namespace + "/" + topic;
}

}  // namespace syncai_costmap_2d
