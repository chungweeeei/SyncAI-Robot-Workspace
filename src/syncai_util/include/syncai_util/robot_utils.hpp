#ifndef SYNCAI_UTIL__ROBOT_UTILS_HPP_
#define SYNCAI_UTIL__ROBOT_UTILS_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_util
{
/**
* @brief get the current pose of the robot
* @param global_pose Pose to transform
* @param tf_buffer TF buffer to use for the transformation
* @param global_frame Frame to transform into
* @param robot_frame Frame to transform from
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
bool getCurrentPose(
  geometry_msgs::msg::PoseStamped & global_pose, tf2_ros::Buffer & tf_buffer,
  const std::string & global_frame = "map", const std::string robot_frame = "base_link",
  const double timeout = 0.1, const rclcpp::Time stamp = rclcpp::Time());

/**
* @brief get an arbitrary pose in a target frame
* @param input_pose Pose to transform
* @param transformed_pose Output transformation
* @param tf_buffer TF buffer to use for the transformation
* @param target_frame Frame to transform into
* @param transform_timeout TF Timeout to use for transformation
* @return bool Whether it could be transformed successfully
*/
bool transformPoseInTargetFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose, tf2_ros::Buffer & tf_buffer,
  const std::string target_frame, const double transform_timeout = 0.1);

// No getTransform() / validateTwist() (2026-10-28): ported from nav2_util,
// never called here.

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__ROBOT_UTILS_HPP_