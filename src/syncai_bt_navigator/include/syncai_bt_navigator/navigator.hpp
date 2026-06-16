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

/**
 * @brief 抽象 navigator interface(定義股價)
 *        - 定義了一個 navigator 要如何被 configure、goal進來要先做什麼檢查等等
 */

namespace syncai_bt_navigator
{

/**
 * @struct FeedbackUtils
 * @brief Navigator feedback utilities required to get transforms and reference frames.
 *        把 「feedback 需要的 TF 相關東西」打包成一包，由 BtNavigator 填好、並傳給 navigator。
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
 * 這邊會設計這個 mutexr 主要是為了限制同時只能有一個 navigator 在跑
 * 因為在 navigation2 中，一個 BtNavigator node上可以掛多個 navigator (例如 nav2裡有 NavigateToPose 和 NavigateThroughPoses)，它們共用同一台機器人。NavigatorMuxer 確保不會兩個同時動。
 */
class NavigatorMuxer
{
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  NavigatorMuxer() : current_navigator_(std::string("")) {}

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
 *        這裡用 template 主要是因為每種 navigator 會綁不同的 action 型別： NavigateToPose 會綁 NavigateToPoseAction，NavigateThroughPoses 會綁 NavigateThroughPosesAction 等等。
 */
template <class ActionT>
class Navigator
{
public:
  using Ptr = std::shared_ptr<syncai_bt_navigator::Navigator<ActionT>>;

  /**
   * @brief A Navigator constructor
   */
  Navigator() { plugin_muxer_ = nullptr; }

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
    rclcpp::Node::WeakPtr parent_node, const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils, syncai_bt_navigator::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<syncai_util::OdomSmoother> odom_smoother)
  {
    // WeakPtr 收進來、在這裡 lock() 成一個區域、短命的 SharedPtr 使用，
    // 避免 navigator 強引用 parent node 造成循環持有。
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    // get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(node);

    // Create the Behavior Tree Action Server for this navigator. Its constructor
    // initializes the action server, loads the default BT and activates it; it
    // throws on failure, so guard against that to preserve the bool contract.
    try {
      bt_action_server_ = std::make_unique<syncai_behavior_tree::BtActionServer<ActionT>>(
        node, getName(), plugin_lib_names, default_bt_xml_filename,
        std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
        std::bind(&Navigator::onLoop, this),
        std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
        std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "Failed to create BT action server: %s", e.what());
      return false;
    }

    // 在 BT.CPP v3 的 Blackboard 底層就是一個 std::unordered_map
    // 也就是說 Blackboard 上可以「 key 把資料送進去 」
    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);                              // NOLINT
    blackboard->set<int>("number_recoveries", 0);                                       // NOLINT
    blackboard->set<std::shared_ptr<syncai_util::OdomSmoother>>(
      "odom_smoother", odom_smoother);  // NOLINT

    // 這裡會呼叫 subclass 的 override 的 configure()。
    // 與就是說在上面 on_configure() function 裡做的事每種 navigator 都會需要的東西 (BtActionServer、共用的 Blackboard key)
    // 而 subclass 自身 override 的 configure() 裡做的事情就是 subclass 各自需要初始化的東西
    // 最後回傳 subclass 初始化後的結果。
    return configure(node, odom_smoother);
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup()
  {
    // 呼叫 subclass 的 override cleanup()
    // 在 subclass 所定義的cleanup()裡所做的事情有包含停下 action_client 和 subscription。
    bool ok = cleanup();
    bt_action_server_.reset();
    return ok;
  }

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   *         getName 會定義在 subclass裡，因為每個 navigator 要 expose 的 action name 不一樣 (NavigateToPose、NavigateThroughPoses 等等)
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
        " rejecting request.",
        getName().c_str());
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
    rclcpp::Node::WeakPtr /*node*/, std::shared_ptr<syncai_util::OdomSmoother> /*odom_smoother*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() { return true; }

  std::unique_ptr<syncai_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Navigator")};
  rclcpp::Clock::SharedPtr clock_;
  FeedbackUtils feedback_utils_;
  NavigatorMuxer * plugin_muxer_;
};

}  // namespace syncai_bt_navigator

#endif  // SYNCAI_BT_NAVIGATOR__NAVIGATOR_HPP_
