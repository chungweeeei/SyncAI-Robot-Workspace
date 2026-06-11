#ifndef SYNCAI_NAV_CORE__PROGRESS_CHECKER_HPP_
#define SYNCAI_NAV_CORE__PROGRESS_CHECKER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace syncai_nav_core
{

/**
 * @class ProgressChecker
 * @brief This class defines the plugin interface used to check the
 * position of the robot to make sure that it is actually progressing
 * towards a goal. Unlike nav2_core, plugins receive a plain rclcpp::Node.
 */
class ProgressChecker
{
public:
  typedef std::shared_ptr<syncai_nav_core::ProgressChecker> Ptr;

  virtual ~ProgressChecker() {}

  /**
   * @brief Initialize parameters for ProgressChecker
   * @param node Node pointer
   * @param plugin_name The name of this checker instance (parameter namespace)
   */
  virtual void initialize(
    const rclcpp::Node::SharedPtr & node, const std::string & plugin_name) = 0;

  /**
   * @brief Checks if the robot has moved compare to previous pose
   * @param current_pose Current pose of the robot
   * @return True if progress is made
   */
  virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;

  /**
   * @brief Reset class state upon calling
   */
  virtual void reset() = 0;
};

}  // namespace syncai_nav_core

#endif  // SYNCAI_NAV_CORE__PROGRESS_CHECKER_HPP_
