#include "syncai_costmap_2d/footprint_subscriber.hpp"

#include <memory>
#include <string>
#include <vector>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace syncai_costmap_2d
{
FootprintSubscriber::FootprintSubscriber(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name, tf2_ros::Buffer & tf,
  std::string robot_base_frame, double transform_timeout)
: tf_(tf), robot_base_frame_(robot_base_frame), transform_tolerance_(transform_timeout)
{
  footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    topic_name, rclcpp::SystemDefaultsQoS(),
    std::bind(&FootprintSubscriber::footprint_callback, this, std::placeholders::_1));
}

bool FootprintSubscriber::getFootprintRaw(
  std::vector<geometry_msgs::msg::Point> & footprint, std_msgs::msg::Header & footprint_header)
{
  if (!footprint_received_) {
    return false;
  }

  auto current_footprint = std::atomic_load(&footprint_);
  footprint =
    toPointVector(std::make_shared<geometry_msgs::msg::Polygon>(current_footprint->polygon));
  footprint_header = current_footprint->header;

  return true;
}

bool FootprintSubscriber::getFootprintInRobotFrame(
  std::vector<geometry_msgs::msg::Point> & footprint, std_msgs::msg::Header & footprint_header)
{
  if (!getFootprintRaw(footprint, footprint_header)) {
    return false;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!syncai_util::getCurrentPose(
        current_pose, tf_, footprint_header.frame_id, robot_base_frame_, transform_tolerance_,
        footprint_header.stamp)) {
    return false;
  }

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);

  std::vector<geometry_msgs::msg::Point> temp;
  transformFootprint(-x, -y, 0, footprint, temp);
  transformFootprint(0, 0, -theta, temp, footprint);

  footprint_header.frame_id = robot_base_frame_;
  footprint_header.stamp = current_pose.header.stamp;

  return true;
}

void FootprintSubscriber::footprint_callback(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::atomic_store(&footprint_, msg);
  if (!footprint_received_) {
    footprint_received_ = true;
  }
}
}  // namespace syncai_costmap_2d