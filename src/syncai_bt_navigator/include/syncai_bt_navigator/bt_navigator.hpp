#ifndef SYNCAI_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
#define SYNCAI_BT_NAVIGATOR__BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_bt_navigator/navigator.hpp"
#include "syncai_bt_navigator/navigators/navigate_to_pose.hpp"
#include "syncai_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_bt_navigator
{

/**
 * @class syncai_bt_navigator::BtNavigator
 * @brief A node that hosts a behavior tree based navigator for navigating a
 * robot to its goal position.
 *
 * @note Unlike nav2 this is a plain rclcpp::Node (no lifecycle). Setup that
 * needs shared_from_this() lives in configure(), which must be called after the
 * node has been created as a shared_ptr.
 */
class BtNavigator : public rclcpp::Node
{
public:
  /**
   * @brief A constructor for syncai_bt_navigator::BtNavigator class
   * @param options Additional options to control creation of the node.
   */
  explicit BtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for syncai_bt_navigator::BtNavigator class
   */
  ~BtNavigator();

  /**
   * @brief Configure the navigator: set up TF, the odometry smoother and the
   * navigate_to_pose navigator. Must be called after the node has been created
   * as a shared_ptr (it relies on shared_from_this()).
   * @return bool If successful
   */
  bool configure();

  /**
   * @brief Release resources allocated in configure()
   * @return bool If successful
   */
  bool cleanup();

protected:
  // To handle all the BT related execution
  std::unique_ptr<syncai_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>> pose_navigator_;
  syncai_bt_navigator::NavigatorMuxer plugin_muxer_;

  // Odometry smoother object
  std::shared_ptr<syncai_util::OdomSmoother> odom_smoother_;

  // Metrics for feedback
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace syncai_bt_navigator

#endif  // SYNCAI_BT_NAVIGATOR__BT_NAVIGATOR_HPP_
