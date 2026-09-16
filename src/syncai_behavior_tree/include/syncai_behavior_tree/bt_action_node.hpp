#ifndef SYNCAI_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
#define SYNCAI_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_behavior_tree/bt_conversions.hpp"

/**
 * @brief Define a base class for BT Action Client Nodes. The Client Node is responsible for sending goals to other action servers.
 */

namespace syncai_behavior_tree
{

using namespace std::chrono_literals;

/**
 * @brief Abstract class representing an action based BT node
 *        Inheritance chain: BtActionNode -> ActionNodeBase -> LeafNode -> TreeNode
 * @tparam ActionT Type of action
 */
template <class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A syncai_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node -> initialises TreeNode::name_, the
   *        name of this node instance; ActionNodeBase::name() returns this tag name afterwards
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   * struct NodeConfiguration{
   *     Blackboard::Ptr blackboard;     // shared blackboard
   *     PortsRemapping input_ports;     // input-port name remapping from the XML
   *     PortsRemapping output_ports;    // output-port name remapping from the XML
   * }
   * - blackboard is the one BtActionServer populated in initialize() with "node",
   *   "server_timeout" and friends.
   * - input_ports / output_ports are PortsRemapping (unordered_map<string,string>) recording the
   *   "port name -> blackboard key" mapping written in the XML as goal="{goal}", which is what
   *   getInput("goal", ...) / setOutput("path", ...) resolve through.
   */
  BtActionNode(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), action_name_(action_name), should_send_goal_(true)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // bt_loop_duration comes from the blackboard, not from the XML.
    // The key is put there by BtActionServer::initialize() (impl:116), and that value in turn
    // comes from a ROS 2 parameter, so as far as bt_action_node is concerned its direct source
    // is the ROS 2 node parameters.
    // Its only use is to derive max_timeout, which caps how long a single blocking wait may
    // block, so that one tick cannot stall the whole tree.
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    // timeout should be less than bt_loop_duration to be able to finish the current tick
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // wait_for_service_timeout is the maximum time the action node waits for the action server
    // at startup; exceeding it is an error.
    // server_timeout is the maximum time to wait for the server's ack / cancel / result after
    // each goal; exceeding it logs a warning and gives up waiting.
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // Initialize the input and output messages
    goal_ = typename ActionT::Goal();
    result_ = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult();

    std::string remapped_action_name;
    if (getInput("server_name", remapped_action_name)) {
      action_name_ = remapped_action_name;
    }
    createActionClient(action_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_INFO(
      node_->get_logger(), "[BtActionNode][%s] \"%s\" BtActionNode initialized", __func__,
      xml_tag_name.c_str());
  }

  // Forbid constructing this object without arguments: writing BtActionNode<X> node; (no
  // arguments) is a compile error.
  // C++ rule: if you declare no constructor at all, the compiler generates a default
  // (no-argument) constructor for you.
  // An instance cannot be built without those three arguments, so the default constructor is
  // deleted outright to keep the compiler from generating a no-argument one.
  BtActionNode() = delete;

  // deconstructor
  virtual ~BtActionNode() {}

  /**
   * @brief Create instance of an "action client"
   * @param action_name Action name to create client for
   *                    Creates the action client and waits for the server to come up
   */
  void createActionClient(const std::string & action_name)
  {
    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = rclcpp_action::create_client<ActionT>(node_, action_name, callback_group_);

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(
      node_->get_logger(), "[BtActionNode][%s] Waiting for \"%s\" action server", __func__,
      action_name.c_str());
    if (!action_client_->wait_for_action_server(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[BtActionNode][%s] \"%s\" action server not available after waiting for %.2fs", __func__,
        action_name.c_str(), wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
        std::string("Action server ") + action_name + std::string(" not available"));
    }
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *         Basic ports (server_name / server_timeout) merged with the subclass's own additions
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("server_name", "Action server name"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   *         The static interface the BT factory requires; by default returns only the basic ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the blackboard
   */
  virtual void on_tick() {}

  /**
   * @brief Function to perform some user-defined operation after a timeout
   * waiting for a result that hasn't been received yet. Also provides access to
   * the latest feedback message from the action server. Feedback will be nullptr
   * in subsequent calls to this function if no new feedback is received while waiting for a result.
   * @param feedback shared_ptr to latest feedback message, nullptr if no new feedback was received
   */
  virtual void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback> /*feedback*/) {}

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the action. Could put a value on the blackboard.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_success() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief Function to perform some user-defined operation when the action is aborted.
   * @return BT::NodeStatus Returns FAILURE by default, user may override return another value
   */
  virtual BT::NodeStatus on_aborted() { return BT::NodeStatus::FAILURE; }

  /**
   * @brief Function to perform some user-defined operation when the action is cancelled.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override return another value
   */
  virtual BT::NodeStatus on_cancelled() { return BT::NodeStatus::SUCCESS; }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   *         tick() is the core state machine, called every round; it mostly returns RUNNING
   */
  BT::NodeStatus tick() override
  {
    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);

      // reset the flag to send the goal or not, allowing the user the option to set it in on_tick
      should_send_goal_ = true;

      // "user defined" callback, may modify "should_send_goal_".
      // on_tick() is where the subclass fills in the goal contents
      on_tick();

      if (!should_send_goal_) {
        return BT::NodeStatus::FAILURE;
      }

      // Send goal_ without blocking and hook up the two callbacks (result / feedback) in advance,
      // so that the spin_some() in later tick()s picks up the replies as they arrive.
      send_new_goal();
    }

    try {
      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      // - future_goal_handle_ is truthy (has a value) => a goal was sent and we are waiting for
      //   the server ack; the goal has not been accepted yet
      // - future_goal_handle_ is falsy (empty) => not waiting for a server ack; usually the ack
      //   is already done (goal accepted), or nothing has been sent at all
      if (future_goal_handle_) {
        auto elapsed =
          (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {
          // return RUNNING if there is still some time before timeout happens
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          // if server has taken more time than the specified timeout value return FAILURE
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // The following code corresponds to the "RUNNING" loop
      if (rclcpp::ok() && !goal_result_available_) {
        // "user defined" callback. May modify the value of "goal_updated_"
        // On every tick(), the feedback the action server sent is handled in on_wait_for_result.
        on_wait_for_result(feedback_);

        // reset feedback to avoid stale information
        // Clear the feedback just handled so the next tick() does not process the same one twice
        feedback_.reset();

        // Check what state the current goal is in on the action server side.
        // If the goal was updated and the server reports it as EXECUTING or ACCEPTED, send the
        // goal again.
        auto goal_status = goal_handle_->get_status();
        if (
          goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                            goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)) {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed =
            (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();

          // If a new goal was just sent and the server has not replied yet, wait for the reply
          // before moving on. Any feedback arriving meanwhile goes through on_wait_for_result.
          if (!is_future_goal_handle_complete(elapsed)) {
            if (elapsed < server_timeout_) {
              return BT::NodeStatus::RUNNING;
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "Timed out while waiting for action server to acknowledge goal request for %s",
              action_name_.c_str());
            future_goal_handle_.reset();
            return BT::NodeStatus::FAILURE;
          }
        }

        // spin_some() is non-blocking: it only runs the callbacks that are ready right now and
        // returns as soon as they are done.
        // Using spin() here would stall this behavior tree tick() completely.
        // spin_some() just peeks: if a feedback / result callback is ready it is handled,
        // otherwise it returns immediately.
        // One important precondition: this uses a dedicated callback group + executor.
        callback_group_executor_.spin_some();

        // check if, after invoking spin_some(), we finally received the result
        if (!goal_result_available_) {
          // Yield this Action, returning RUNNING
          return BT::NodeStatus::RUNNING;
        }
      }
    } catch (const std::runtime_error & e) {
      if (
        e.what() == std::string("send_goal failed") ||
        e.what() == std::string("Goal was rejected by the action server")) {
        // Action related failure that should not fail the tree, but the node
        return BT::NodeStatus::FAILURE;
      } else {
        // Internal exception to propagate to the tree
        throw e;
      }
    }

    // The action server's result has arrived; the result code decides this BT node's final status.
    BT::NodeStatus status;
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        status = on_success();
        break;

      case rclcpp_action::ResultCode::ABORTED:
        status = on_aborted();
        break;

      case rclcpp_action::ResultCode::CANCELED:
        status = on_cancelled();
        break;

      default:
        throw std::logic_error("BtActionNode::Tick: invalid status value");
    }

    goal_handle_.reset();
    return status;
  }

  /**
   * @brief The other (optional) override required by a BT action. In this case, we
   * make sure to cancel the ROS2 action if it is still running.
   */
  void halt() override
  {
    // halt() is called when this TreeNode is still RUNNING but gets interrupted from outside:
    // - A new navigation goal arrived and the whole tree must terminate and start over.
    // - A higher-priority branch of a ReactiveFallback / Parallel fired and the branch that is
    //   running has to yield.
    // - haltTree() was called from above (BtActionServer does this on cancel).

    // Check whether there is a goal still RUNNING that needs cancelling
    if (should_cancel_goal()) {
      auto future_result =
        action_client_->async_get_result(goal_handle_);  // ask for the result first
      auto future_cancel =
        action_client_->async_cancel_goal(goal_handle_);  // then send the cancel request

      // Wait for the cancel to complete
      if (
        callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to cancel action server for %s", action_name_.c_str());
      }

      // Wait for the result to come back
      if (
        callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to get result for %s in node halt!", action_name_.c_str());
      }

      on_cancelled();  // on_cancelled() is the hook for the subclass
    }

    // Reset the status back to IDLE
    setStatus(BT::NodeStatus::IDLE);
  }

protected:
  /**
   * @brief Function to check if current goal should be cancelled
   * @return bool True if current goal should be cancelled, false otherwise
   */
  bool should_cancel_goal()
  {
    // Shut the node down if it is currently running
    if (status() != BT::NodeStatus::RUNNING) {
      return false;
    }

    // No need to cancel the goal if goal handle is invalid
    if (!goal_handle_) {
      return false;
    }

    callback_group_executor_.spin_some();
    auto status = goal_handle_->get_status();

    // Check if the goal is still executing
    return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
           status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  }

  /**
   * @brief Function to send new goal to action server
   */
  void send_new_goal()
  {
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result) {
        if (future_goal_handle_) {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "[BtActionNode][%s] Goal result for %s available, but it hasn't received the goal "
            "response yet. "
            "It's probably a goal result for the last goal request",
            __func__, action_name_.c_str());
          return;
        }

        // TODO(#1652): a work around until rcl_action interface is updated
        // if goal ids are not matched, the older goal call this callback so ignore the result
        // if matched, it must be processed (including aborted)
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };

    send_goal_options.feedback_callback =
      [this](
        typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
        const std::shared_ptr<const typename ActionT::Feedback> feedback) { feedback_ = feedback; };

    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));
    time_goal_sent_ = node_->now();
  }

  /**
   * @brief Function to check if the action server acknowledged a new goal
   * @param elapsed Duration since the last goal was sent and future goal handle has not completed.
   * After waiting for the future to complete, this value is incremented with the timeout value.
   * @return boolean True if future_goal_handle_ returns SUCCESS, False otherwise
   */
  bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;

    // server has already timed out, no need to sleep
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }

    auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    elapsed += timeout;

    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }

    return false;
  }

  /**
   * @brief Function to increment recovery count on blackboard if this node wraps a recovery
   */
  void increment_recovery_count()
  {
    int recovery_count = 0;
    config().blackboard->template get<int>("number_recoveries", recovery_count);  // NOLINT
    recovery_count += 1;
    config().blackboard->template set<int>("number_recoveries", recovery_count);  // NOLINT
  }

  std::string action_name_;
  typename std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;

  // All ROS2 actions have a goal and a result
  typename ActionT::Goal goal_;
  bool goal_updated_{false};
  bool goal_result_available_{false};
  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // To handle feedback from action server
  std::shared_ptr<const typename ActionT::Feedback> feedback_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while waiting for response from a server when a
  // new action goal is sent or canceled
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds max_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // To track the action server acknowledgement when a new goal is sent
  std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr>>
    future_goal_handle_;
  rclcpp::Time time_goal_sent_;

  // Can be set in on_tick or on_wait_for_result to indicate if a goal should be sent.
  bool should_send_goal_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
