#ifndef SYNCAI_NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_

#include "geometry_msgs/msg/polygon.h"
#include "geometry_msgs/msg/polygon_stamped.h"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_publisher.hpp"

namespace syncai_costmap_2d
{
class Costmap2DROS : public rclcpp::Node
{
public:
  Costmap2DROS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  explicit Costmap2DROS(const std::string & name);
  explicit Costmap2DROS(
    const std::string & name, const std::string & parent_namespace,
    const std::string & local_namespace);

  ~Costmap2DROS();

private:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;

  std::unique_ptr<Costmap2DPublisher> costmap_2d_publisher_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

  // Dedicated callback group and executor for tf timer_interface and message fillter
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  // std::unique_ptr<>
};
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D__COSTMAP_2D_ROS_HPP_