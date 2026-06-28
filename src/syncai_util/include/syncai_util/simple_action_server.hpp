#ifndef SYNCAI_UTIL__SIMPLE_ACTION_SERVER_HPP_
#define SYNCAI_UTIL__SIMPLE_ACTION_SERVER_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "syncai_util/node_thread.hpp"

namespace syncai_util
{
/**
 * @class syncai_util::SimpleActionServer
 * @brief An action server wrapper to make applications simpler using Actions
 */
template <typename ActionT>
class SimpleActionServer
{
public:
  typedef std::function<void()> ExecuteCallback;
  typedef std::function<void()> CompletionCallback;

  /**
   * @brief A constructor for SimpleActionServer
   * @param node Ptr to node to make actions
   * @param action_name Name of the action to call
   * @param execute_callback Execution callback function of Action
   * @param server_timeout Timeout to react to stop or preemption requests
   * @param spin_thread Whether to spin with a dedicated thread internally
   * @param options Options to pass to the underlying rcl_action_server_t
   */
  template <typename NodeT>
  explicit SimpleActionServer(
    NodeT node, const std::string & action_name, ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
  : SimpleActionServer(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(), action_name,
      execute_callback, completion_callback, server_timeout, spin_thread, options)
  {
  }

  /**
   * @brief A constructor for SimpleActionServer
   * @param <node interfaces> Abstract node interfaces to make actions
   * @param action_name Name of the action to call
   * @param execute_callback Execution callback function of Action
   * @param server_timeout Timeout to react to stop or preemption requests
   * @param spin_thread Whether to spin with a dedicated thread internally
   * @param options Options to pass to the underlying rcl_action_server_t
   */
  explicit SimpleActionServer(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    const std::string & action_name, ExecuteCallback execute_callback,
    CompletionCallback completion_callback = nullptr,
    std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000),
    bool spin_thread = false,
    const rcl_action_server_options_t & options = rcl_action_server_get_default_options())
  : node_base_interface_(node_base_interface),
    node_clock_interface_(node_clock_interface),
    node_logging_interface_(node_logging_interface),
    node_waitables_interface_(node_waitables_interface),
    action_name_(action_name),
    execute_callback_(execute_callback),
    completion_callback_(completion_callback),
    server_timeout_(server_timeout),
    spin_thread_(spin_thread)
  {
    if (spin_thread_) {
      // Create a callback group for the action server to be added to the executor
      callback_group_ = node_base_interface->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }

    action_server_ = rclcpp_action::create_server<ActionT>(
      node_base_interface_, node_clock_interface_, node_logging_interface_,
      node_waitables_interface_, action_name_,
      std::bind(
        &SimpleActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SimpleActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&SimpleActionServer::handle_accepted, this, std::placeholders::_1), options,
      callback_group_);

    if (spin_thread_) {
      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_callback_group(callback_group_, node_base_interface_);
      executor_thread_ = std::make_unique<syncai_util::NodeThread>(executor_);
    }
  }

  /**
   * @brief handle the goal requested: accept or reject. This implementation always accepts.
   * @param uuid Goal ID
   * @param Goal A shared pointer to the specific goal
   * @return GoalResponse response of the goal processed
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const typename ActionT::Goal> /*goal*/)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!server_active_) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(),
      "[ActionServer][%s] Received request for goal acceptance", __func__);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * @brief Accepts cancellation requests of action server.
   * @param uuid Goal ID
   * @param Goal A server goal handle to cancel
   * @return CancelResponse response of the goal cancelled
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!handle->is_active()) {
      RCLCPP_WARN(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Received request for goal cancellation, but the handle is inactive, so "
        "reject the request",
        __func__);
      return rclcpp_action::CancelResponse::REJECT;
    }

    RCLCPP_INFO(
      node_logging_interface_->get_logger(),
      "[ActionServer][%s] Received request for goal cancellation", __func__);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /**
   * @brief Handles accepted goals and adds to preempted queue to switch to
   * @param Goal A server goal handle to cancel
   */
  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(), "[ActionServer][%s] Handling accepted goal", __func__);

    if (is_active(current_handle_) || is_running()) {
      RCLCPP_DEBUG(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] An older goal is active, moving the new goal to a pending slot.",
        __func__);
      // 已經有一個 action goal 正在執行了 -> 新的 goal 放進 pending slot 等待
      if (is_active(pending_handle_)) {
        RCLCPP_DEBUG(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] The pending slot is occupied. The previous pending goal will be "
          "terminated and replaced.",
          __func__);
        terminate(pending_handle_);  // pending 已被占用 -> 舊的 pending 直接終結，最新的贏
      }
      pending_handle_ = handle;  // pending slot 放入新的 goal handle
      preempt_requested_ = true;

    } else {
      if (is_active(current_handle_)) {
        RCLCPP_ERROR(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] Forgot to handle a preemption. Terminating the pending goal.",
          __func__);
        terminate(pending_handle_);
        preempt_requested_ = false;
      }

      // 沒有正在執行的 goal 了 -> 直接成為 current，開一個
      current_handle_ = handle;
      // Return quickly to avoid blocking the executor, so spin up a new thread
      RCLCPP_DEBUG(
        node_logging_interface_->get_logger(), "[ActionServer][%s] Executing goal asynchronously.",
        __func__);

      /**
       * std::async 有兩種啟動策略：
       * std::async(std::launch::async, func) 保證「如同在新的thread」立刻開始執行 func
       * std::async(std::launch::deferred, func) 完全不開thread，直到有人對future 呼叫 .get()/.wait()時，才在「呼叫者的thread」上同步執行
       * std::async(f) 預設
       */
      execution_future_ = std::async(std::launch::async, [this]() { work(); });
    }
  }

  void work()
  {
    /** 
     * 這裡的 while loop主要負責
     * 1. 檢查是否需要 stop execution
     * 2. 檢查是否有正在執行的goal handle
     * 3. 每一個 loop 代表處理完一個 action goal
     */
    while (rclcpp::ok() && !stop_execution_ && is_active(current_handle_)) {
      RCLCPP_DEBUG(
        node_logging_interface_->get_logger(), "[ActionServer][%s] Executing the goal...",
        __func__);
      try {
        execute_callback_();
      } catch (std::exception & ex) {
        RCLCPP_ERROR(
          node_logging_interface_->get_logger(),
          "Action server failed while executing action callback: \"%s\"", ex.what());
        terminate_all();
        if (completion_callback_) {
          completion_callback_();
        }
        return;
      }

      RCLCPP_DEBUG(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Blocking processing of new goal handles.", __func__);
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);

      if (stop_execution_) {
        RCLCPP_WARN(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] Stopping the thread per request.", __func__);
        terminate_all();
        if (completion_callback_) {
          completion_callback_();
        }
        break;
      }

      if (is_active(current_handle_)) {
        RCLCPP_WARN(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] Current goal was not completed successfully. Terminating it.",
          __func__);
        terminate(current_handle_);
        if (completion_callback_) {
          completion_callback_();
        }
      }

      if (is_active(pending_handle_)) {
        RCLCPP_DEBUG(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] Executing a pending handle on the existing thread.", __func__);
        accept_pending_goal();
      } else {
        RCLCPP_DEBUG(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] Done processing available goals.", __func__);
        break;
      }
    }
    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(), "[ActionServer][%s] Worker thread done.", __func__);
  }

  /**
   * @brief Active action server
   */
  void activate()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    server_active_ = true;
    stop_execution_ = false;
  }

  /**
   * @brief Deactivate action server
   */
  void deactivate()
  {
    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(), "[ActionServer][%s] Deactivating...", __func__);

    {
      std::lock_guard<std::recursive_mutex> lock(update_mutex_);
      server_active_ = false;
      stop_execution_ = true;
    }

    if (!execution_future_.valid()) {
      return;
    }

    if (is_running()) {
      RCLCPP_WARN(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Requested to deactivate server but goal is still executing."
        " Should check if action server is running before deactivating.",
        __func__);
    }

    using namespace std::chrono;  //NOLINT
    auto start_time = steady_clock::now();
    while (execution_future_.wait_for(milliseconds(100)) != std::future_status::ready) {
      RCLCPP_INFO(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Waiting for async process to finish.", __func__);
      if (steady_clock::now() - start_time >= server_timeout_) {
        terminate_all();
        if (completion_callback_) {
          completion_callback_();
        }
        RCLCPP_ERROR(
          node_logging_interface_->get_logger(),
          "[ActionServer][%s] Action callback is still running and missed deadline to stop.",
          __func__);
      }
    }

    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(), "[ActionServer][%s] Deactivation completed.",
      __func__);
  }

  /**
   * @brief Whether the action server is munching on a goal
   * @return bool If its running or not
   */
  bool is_running()
  {
    return execution_future_.valid() && (execution_future_.wait_for(std::chrono::milliseconds(0)) ==
                                         std::future_status::timeout);
  }

  /**
   * @brief Whether the action server is active or not
   * @return bool If its active or not
   */
  bool is_server_active()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return server_active_;
  }

  /**
   * @brief Whether the action server has been asked to be preempted with a new goal
   * @return bool If there's a preemption request or not
   */
  bool is_preempt_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

  /**
   * @brief Accept pending goals
   * @return Goal Ptr to the  goal that's going to be accepted
   */
  const std::shared_ptr<const typename ActionT::Goal> accept_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Attempting to get pending goal when not available.", __func__);
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      RCLCPP_DEBUG(
        node_logging_interface_->get_logger(), "[ActionServer][%s] Cancelling the previous goal.",
        __func__);
      current_handle_->abort(empty_result());
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(), "[ActionServer][%s] Preempted goal.", __func__);

    return current_handle_->get_goal();
  }

  /**
   * @brief Terminate pending goals
   */
  void terminate_pending_goal()
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Attempting to terminate pending goal when not available.", __func__);
      return;
    }

    terminate(pending_handle_);
    preempt_requested_ = false;

    RCLCPP_DEBUG(
      node_logging_interface_->get_logger(), "[ActionServer][%s] Pending goal terminated.",
      __func__);
  }

  /**
   * @brief Get the current goal object
   * @return Goal Ptr to the  goal that's being processed currently
   */
  const std::shared_ptr<const typename ActionT::Goal> get_current_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] A goal is not available or has reached a final state.", __func__);
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return current_handle_->get_goal();
  }

  const rclcpp_action::GoalUUID get_current_goal_id() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(current_handle_)) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] A goal is not available or has reached a final state.", __func__);
      return rclcpp_action::GoalUUID();
    }

    return current_handle_->get_goal_id();
  }

  /**
   * @brief Get the pending goal object
   * @return Goal Ptr to the goal that's pending
   */
  const std::shared_ptr<const typename ActionT::Goal> get_pending_goal() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Attempting to get pending goal when not available.", __func__);
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    return pending_handle_->get_goal();
  }

  /**
   * @brief Whether or not a cancel command has come in
   * @return bool Whether a cancel command has been requested or not
   */
  bool is_cancel_requested() const
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    // A cancel request is assumed if either handle is canceled by the client.

    if (current_handle_ == nullptr) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Checking for cancel but current goal is not available.", __func__);
      return false;
    }

    if (pending_handle_ != nullptr) {
      return pending_handle_->is_canceling();
    }

    return current_handle_->is_canceling();
  }

  /**
   * @brief Terminate all pending and active actions
   * @param result A result object to send to the terminated actions
   */
  void terminate_all(
    typename std::shared_ptr<typename ActionT::Result> result =
      std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }

  /**
   * @brief Terminate the active action
   * @param result A result object to send to the terminated action
   */
  void terminate_current(
    typename std::shared_ptr<typename ActionT::Result> result =
      std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
  }

  /**
   * @brief Return success of the active action
   * @param result A result object to send to the terminated actions
   */
  void succeeded_current(
    typename std::shared_ptr<typename ActionT::Result> result =
      std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (is_active(current_handle_)) {
      RCLCPP_DEBUG(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Setting succeed on current goal.", __func__);
      current_handle_->succeed(result);
      current_handle_.reset();
    }
  }

  /**
   * @brief Publish feedback to the action server clients
   * @param feedback A feedback object to send to the clients
   */
  void publish_feedback(typename std::shared_ptr<typename ActionT::Feedback> feedback)
  {
    if (!is_active(current_handle_)) {
      RCLCPP_ERROR(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Trying to publish feedback when the current goal handle is not active.",
        __func__);
      return;
    }

    current_handle_->publish_feedback(feedback);
  }

protected:
  // The SimpleActionServer isn't itself a node, so it needs interfaces to one
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface_;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface_;

  std::string action_name_;

  ExecuteCallback execute_callback_;
  CompletionCallback completion_callback_;
  std::future<void> execution_future_;
  bool stop_execution_{false};

  // 這裡的 mutex 只要是用來防止 race condition 的發生
  // Thread A: spin thread => handle_goal / handle_cancel / handle_accepted
  // Thread B: std::async worker => work()、以及 callback 裡呼叫的 succeeded_current / publish_feedback / is_cancel_requested / accept_pending_goal ...
  // 共享的 variable 主要有
  // - current_handle_、 pending_handle_
  // - preempt_requested_ 、 server_active_ 、 stop_execution_
  mutable std::recursive_mutex update_mutex_;

  bool server_active_{false};
  bool preempt_requested_{false};
  std::chrono::milliseconds server_timeout_;

  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> pending_handle_;

  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  bool spin_thread_;
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<syncai_util::NodeThread> executor_thread_;

  /**
   * @brief Generate an empty result object for an action type
   */
  constexpr auto empty_result() const { return std::make_shared<typename ActionT::Result>(); }

  /**
   * @brief Whether a given goal handle is currently active
   * @param handle Goal handle to check
   * @return Whether this goal handle is active
   */
  constexpr bool is_active(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const
  {
    return handle != nullptr && handle->is_active();
  }

  /**
   * @brief Terminate a particular action with a result
   * @param handle goal handle to terminate
   * @param the Results object to terminate the action with
   */
  void terminate(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & handle,
    typename std::shared_ptr<typename ActionT::Result> result =
      std::make_shared<typename ActionT::Result>())
  {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!is_active(handle)) {
      return;
    }

    if (handle->is_canceling()) {
      RCLCPP_INFO(
        node_logging_interface_->get_logger(),
        "[ActionServer][%s] Client requested to cancel the goal. Cancelling.", __func__);
      handle->canceled(result);
    } else {
      RCLCPP_WARN(
        node_logging_interface_->get_logger(), "[ActionServer][%s] Aborting handle.", __func__);
      handle->abort(result);
    }

    handle.reset();
  }
};

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__SIMPLE_ACTION_SERVER_HPP_