#ifndef SYNCAI_BT_NAVIGATOR__NAVIGATOR_HPP_
#define SYNCAI_BT_NAVIGATOR__NAVIGATOR_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/bt_action_server.hpp"
#include "syncai_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_bt_navigator
{

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 */
struct FeedbackUtils
{
  std::string robot_frame;
  std::string global_frame;
  double transform_tolerance;
  std::shared_ptr<tf2_ros::Buffer> tf;
};

/**
 * @class NavigatorMuxer
 * @brief A class to control the state of the BT navigator by allowing only a single
 * plugin to be processed at a time.
 */
class NavigatorMuxer
{
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  NavigatorMuxer()
  : current_navigator_(std::string("")) {}

  /**
   * @brief Get the navigator muxer state
   * @return bool If a navigator is in progress
   */
  bool isNavigating()
  {
    std::scoped_lock l(mutex_);
    return !current_navigator_.empty();
  }

  /**
   * @brief Start navigating with a given navigator
   * @param string Name of the navigator to start
   */
  void startNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation requested while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
    }
    current_navigator_ = navigator_name;
  }

  /**
   * @brief Stop navigating with a given navigator
   * @param string Name of the navigator ending task
   */
  void stopNavigating(const std::string & navigator_name)
  {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("NavigatorMutex"),
        "Major error! Navigation stopped while another navigation"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a navigator plugin.");
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
 * @brief Navigator interface that acts as a base class for all BT-based Navigator action's plugins
 */
template <class ActionT>
class Navigator
{
public:
  using Ptr = std::shared_ptr<syncai_bt_navigator::Navigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  Navigator()
  {
    plugin_muxer_ = nullptr;
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~Navigator() = default;

  /**
   * @brief Configuration to setup the navigator's backend BT and actions
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @param feedback_utils Some utilities useful for navigators to have
   * @param plugin_muxer The muxing object to ensure only one navigator
   * can be active at a time
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool If successful
   *
   * @note Unlike nav2, the BtActionServer here initializes itself (creates the
   * action server, loads the default BT and activates) inside its constructor,
   * so there is no separate on_configure()/on_activate() step on the server.
   */
  bool on_configure(
    rclcpp::Node::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    syncai_bt_navigator::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<syncai_util::OdomSmoother> odom_smoother)
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    // get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // Create the Behavior Tree Action Server for this navigator. Its constructor
    // initializes the action server, loads the default BT and activates it; it
    // throws on failure, so guard against that to preserve the bool contract.
    try {
      bt_action_server_ = std::make_unique<syncai_behavior_tree::BtActionServer<ActionT>>(
        node,
        getName(),
        plugin_lib_names,
        default_bt_xml_filename,
        std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
        std::bind(&Navigator::onLoop, this),
        std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
        std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to create BT action server: %s", e.what());
      return false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard->set<int>("number_recoveries", 0);  // NOLINT
    blackboard->set<std::shared_ptr<syncai_util::OdomSmoother>>(
      "odom_smoother", odom_smoother);  // NOLINT

    return configure(parent_node, odom_smoother);
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup()
  {
    bool ok = cleanup();
    bt_action_server_.reset();
    return ok;
  }

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

  /**
   * @brief Get the default behavior tree filepath for this navigator
   * @return string Filepath to default XML
   */
  virtual std::string getDefaultBTFilepath(rclcpp::Node::WeakPtr node) = 0;

  /**
   * @brief Get the action server
   * @return Action server pointer
   */
  std::unique_ptr<syncai_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(
        logger_,
        "Requested navigation from %s while another navigator is processing,"
        " rejecting request.", getName().c_str());
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function to mux navigators
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status)
  {
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const syncai_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param Method to configure resources.
   */
  virtual bool configure(
    rclcpp::Node::WeakPtr /*node*/,
    std::shared_ptr<syncai_util::OdomSmoother>/*odom_smoother*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() {return true;}

  std::unique_ptr<syncai_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;
};

}  // namespace syncai_bt_navigator

#endif  // SYNCAI_BT_NAVIGATOR__NAVIGATOR_HPP_
