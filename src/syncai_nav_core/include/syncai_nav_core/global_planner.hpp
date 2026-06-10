#ifndef SYNCAI_CORE__GLOBAL_PLANNER_HPP_
#define SYNCAI_CORE__GLOBAL_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_nav_core
{

class GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<GlobalPlanner>;

  virtual ~GlobalPlanner() {};

  virtual void configure(
    const rclcpp::Node::SharedPtr & node, std::string & name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) = 0;

  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
}
}  // namespace syncai_nav_core

#endif