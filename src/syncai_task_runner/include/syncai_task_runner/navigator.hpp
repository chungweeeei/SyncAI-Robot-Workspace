#ifndef SYNCAI_TASK_RUNNER__NAVIGATOR_HPP_
#define SYNCAI_TASK_RUNNER__NAVIGATOR_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/bt_action_server.hpp"
#include "syncai_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_task_runner
{

/**
 * @struct FeedbackUtils
 * @brief TF-related utilities a navigator needs to compute feedback. Filled in
 * by the host node (TaskRunner) and handed to each navigator.
 */
struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @class NavigatorMutex
 * @brief Ensures only a single navigator runs at a time, so two tasks never
 * drive the same robot concurrently.
 */
class NavigatorMutex
{
public:
  NavigatorMutex() : current_navigator_(std::string("")) {}

  bool isNavigating()
  {
    std::scoped_lock l(mutex_);
    return !current_navigator_.empty();
  }

  void startNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "[NavigatorMutex][%s] Major error! Navigation requested while another navigation"
        " task is in progress!",
        __func__);
    }
    current_navigator_ = navigator_name;
  }

  void stopNavigating()
  {
    std::scoped_lock l(mutex_);
    if (current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "[NavigatorMutex][%s] Major error! Navigation stopped while no navigation"
        " task is in progress!",
        __func__);
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

/**
 * @class Navigator
 * @brief Base class for all BT-based navigators. Templated on the action type
 * because each navigator binds a different action (NavigateToPose, etc.).
 */
template <class ActionT>
class Navigator
{
public:
  using Ptr = std::shared_ptr<syncai_task_runner::Navigator<ActionT>>;

  Navigator() { plugin_mutex_ = nullptr; }

  virtual ~Navigator() = default;

  bool on_initialize(
    rclcpp::Node::WeakPtr parent_node, const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils, NavigatorMutex * plugin_mutex,
    std::shared_ptr<syncai_util::OdomSmoother> odom_smoother)
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_mutex_ = plugin_mutex;

    // get the default behavior tree for the navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(node);

    // Create the Behavior Tree Action Server for this navigator. Its constructor
    // creates the action server, loads the default BT and activates it, throwing
    // on failure, so guard against that to preserve the bool contract.
    try {
      bt_action_server_ = std::make_unique<syncai_behavior_tree::BtActionServer<ActionT>>(
        node, getName(), plugin_lib_names, default_bt_xml_filename,
        std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
        std::bind(&Navigator::onLoop, this),
        std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
        std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "[Navigator][%s] Failed to create BT action server: %s", __func__, e.what());
      return false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);
    blackboard->set<bool>("initial_pose_received", false);
    blackboard->set<int>("number_recoveries", 0);
    blackboard->set<std::shared_ptr<syncai_util::OdomSmoother>>("odom_smoother", odom_smoother);

    return configure(node, odom_smoother);
  }

  bool on_cleanup()
  {
    bool ok = cleanup();
    bt_action_server_.reset();
    return ok;
  }

  virtual std::string getName() = 0;
  virtual std::string getDefaultBTFilepath(rclcpp::Node::WeakPtr node) = 0;

  // get action server pointer
  std::unique_ptr<syncai_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  /**
   * @brief Intermediate goal reception that muxes navigators: reject the goal
   * if another navigator is already running, otherwise claim the mutex.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_mutex_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "[Navigator][%s] Requested navigation from %s while another navigator is processing,"
        " rejecting request.",
        __func__, getName().c_str());
      return false;
    }

    // let the subclass decide whether the goal is valid and can be processed
    bool goal_accepted = goalReceived(goal);

    // claim the mutex so no other navigator accepts a goal while this one runs
    if (goal_accepted) {
      plugin_mutex_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief Intermediate completion that releases the mutex.
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status)
  {
    plugin_mutex_->stopNavigating();
    goalCompleted(result, final_bt_status);
  }

  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status) = 0;

  virtual void onLoop() = 0;
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  virtual bool configure(
    rclcpp::Node::WeakPtr /*node*/, std::shared_ptr<syncai_util::OdomSmoother> /*odom_smoother*/)
  {
    return true;
  }

  virtual bool cleanup() { return true; }

  std::unique_ptr<syncai_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMutex * plugin_mutex_;
};

}  // namespace syncai_task_runner

#endif  // SYNCAI_TASK_RUNNER__NAVIGATOR_HPP_
