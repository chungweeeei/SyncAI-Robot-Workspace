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
 *        BtActionNode 繼承的chain是: BtActionNode -> ActionNodeBase -> LeafNode -> TreeNode
 * @tparam ActionT Type of action
 */
template <class ActionT>
class BtActionNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A syncai_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node -> 初始化 TreeNode 的 name_，是這個節點 instance 的名稱，之後透過ActionNodeBase的 name() 就可以獲取這個 tag name
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   * struct NodeConfiguration{
   *     Blackboard::Ptr blackboard;     // 共享 blackboard
   *     PortsRemapping input_ports;     // XML 上 input port 的名稱對應
   *     PortsRemapping output_ports;    // XML 上 output port 的名稱對應
   * }
   * - blackboard 是 BtActionServer 在 initialize() 時塞進 node、server_timeout 等值的那塊blackboard上。
   * - input_ports / output_ports 是 PortsRemapping(unordered_map<string,string>)，記錄 XML 裡 goal="{goal}"
   *   這種「port 名 -> blackboard key」的對應。 getInput("goal", ...) / setOutput("path", ...)
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

    // bt_loop_duration 這個參數是從 blackboard 拿的並不是從 XML 獲取的。
    // 而這個 key 是 BtActionServer::initialize()(impl:116)放進去的，而那個值又源自於 ROS2 參數。
    // 所以對 bt_action_node 來說，它的直接根源就是 ROS2 node parameters
    // 唯一用途：算出 max_timeout => 用途就是限制「單次 blocking wait」最長能阻塞多久，避免一個tick把整棵樹卡住。
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");
    // timeout should be less than bt_loop_duration to be able to finish the current tick
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // wait_for_service_timeout 這個參數是 action node 啟動時等待 action server 的最大時間，超過就報錯。
    // server_timeout 這個參數是每個 action goal 後等 server 「 ack / cancel / result 」的最大時間，超過就報警告並放棄等待。
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

  // 禁止無參數 construct 這個 object。也就是若寫 BtActionNode<X> node(不給參數)的話，compile會直接報錯。
  // 在 C++ 中有個規則： 如果你沒有宣告任何 constructor 的話，compiler 會自動幫你生成一個 default constructor (無參數版)。
  // 因爲物件中如果少了這三個參數就沒辦法建立instance，所以直接刪除掉 default constructor，讓 compiler 不會幫你生成一個無參數的 constructor。
  BtActionNode() = delete;

  // deconstructor
  virtual ~BtActionNode() {}

  /**
   * @brief Create instance of an "action client"
   * @param action_name Action name to create client for
   *                    建立 action client + 等 server 上線
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
   *         提供基本的 port (server_name / server_timeout) 再併入自定義的subclass裡
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
   *         BT factory要的static interface，預設只回basic ports
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
   *         tick() 為核心狀態機，每round被呼叫，大多回 RUNNING
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
      // on_tick() 就是讓 subclass 填寫 goal 內容的地方
      on_tick();

      if (!should_send_goal_) {
        return BT::NodeStatus::FAILURE;
      }

      // non-blocking 的將 goal_ 送出去，並預先掛好兩個 callback function ( result / feedback)，讓之後 tick() 裡的
      // spin_some() 一處發就能夠收到回應。
      send_new_goal();
    }

    try {
      // if new goal was sent and action server has not yet responded
      // check the future goal handle
      // - future_goal_handle_ 是 true(有值) => 「送了 goal，正在等 server ack， goal還沒被accept」
      // - future_goal_handle_ 是 false(沒值) => 「沒有在等 server ack」，通常代表 ack 已經完成(goal 已經被 accept)， 或根本還沒送
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
        // 每一輪的 tick()， action server 回覆的 feedback 都會在 on_wait_for_result function 裡進行處理。
        on_wait_for_result(feedback_);

        // reset feedback to avoid stale information
        // 把已經處理過的 feedback 清掉，避免下一輪 tick() 又處理到同一筆 feedback
        feedback_.reset();

        // 確認當前的 goal 在 action server 端目前是處於什麼狀態。
        // 如果 goal 有換新了，且目前 action server 的狀態是「正在執行」或「已經接受」，就再送一次 goal。
        auto goal_status = goal_handle_->get_status();
        if (
          goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                            goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED)) {
          goal_updated_ = false;
          send_new_goal();
          auto elapsed =
            (node_->now() - time_goal_sent_).template to_chrono<std::chrono::milliseconds>();

          // 如果剛送出新 goal，且 server 還沒回應，就先等 server 回應再繼續往下走。等的過程中如果有新的 feedback 就丟到 on_wait_for_result 處理。
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

        // spin_some() 是 non_blocking，只處理「此刻已就緒」的 callback function，處理完後就立刻return。
        // 這裡如果用 spin() 的話，那這個 behavior tree tick() 就會完全卡住。
        // 用 spin_some()： 只「撈一下」目前有沒有 feedback / result callback ready，若有就處理，沒有就直接 return。
        // 一個重要前提:這裡用的是「獨立的 callback group + executor」
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

    // 已經拿到 action server 回傳的 result，根據 result code 來決定這個 BT Node的最終回傳狀態。
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
    // 這裡的 halt() 是 TreeNod 還在 RUNNING，卻被「從外面打斷」時呼叫的。
    // - 收到新的導航目標，整顆Tree要terminate重來。
    // - 某個 ReactiveFallback / Parallel 裡的「更高優先分支」被觸發，要求正在跑的這個分支讓位。
    // - 上層 haltTree() 被呼叫 (BtActionServer 在 cancel 時會做)。

    // 確認當前是否有「還在 RUNNING 的 goal」要取消
    if (should_cancel_goal()) {
      auto future_result =
        action_client_->async_get_result(goal_handle_);  // action_client 先要結果
      auto future_cancel =
        action_client_->async_cancel_goal(goal_handle_);  // action client 送出 cancel request

      // 等 cancel 完成
      if (
        callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to cancel action server for %s", action_name_.c_str());
      }

      // 等 result 回來
      if (
        callback_group_executor_.spin_until_future_complete(future_result, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(
          node_->get_logger(), "Failed to get result for %s in node halt!", action_name_.c_str());
      }

      on_cancelled();  // on_cancelled() 是給 subclass 用的 callback function
    }

    // 狀態重置回 IDLE
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
