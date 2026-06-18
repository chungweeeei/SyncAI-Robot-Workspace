#ifndef SYNCAI_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_
#define SYNCAI_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "syncai_behavior_tree/bt_conversions.hpp"

namespace syncai_behavior_tree
{
using namespace std::chrono_literals;

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 * @details ActionNodeBase就是不靠 BehaviorTree的框架，自己手寫 tick() + halt()
 *          用 template<class ServiceT> 主要是讓這個 class 可以被用來建立不同 service type 的 BT Node
 */
template <class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
  /**
   * @brief A syncai_behavior_tree::BtServiceNode constructor
   * @param service_node_name BT node name
   * @param conf BT node configuration
   * @param service_name Optional service name this node creates a client for instead of from input port
   */
  BtServiceNode(
    const std::string & service_node_name, const BT::NodeConfiguration & conf,
    const std::string & service_name = "")
  : BT::ActionNodeBase(service_node_name, conf),
    service_name_(service_name),
    service_node_name_(service_node_name)
  {
    // config() 是從 BT::TreeNode 繼承來的method，function會回傳這個 node 的 NodeConfiguration也就是constructor收到的那個conf
    // 這裡 ->template 是在告訴compiler 「config()回傳的 NodeConfiguration裡的 blackboard 是一個 template type」
    // 因為 blackboard 是一個 pointer to a template class Blackboard::Ptr
    // - Get rclcpp::Node shared pointer from the blackboard
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    // Get the required items from the blackboard
    auto bt_loop_duration =
      config().blackboard->template get<std::chrono::milliseconds>("bt_loop_duration");

    // 整個 service call 的總時長（從送出 request 開始計算到放棄）
    server_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
    getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
    wait_for_service_timeout_ =
      config().blackboard->template get<std::chrono::milliseconds>("wait_for_service_timeout");

    // 單次 tick 最多 spin 多久，每次 tick 不能無限期卡住 tree thread，否則 run() 迴圈轉不動（不能檢查 cancel、不能跑 onLoop）。
    // 所以限制「每 tick 最多花半個 loop 週期去等」，等不到就先回RUNNING、下一圈再來。
    max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(bt_loop_duration * 0.5);

    // Now that we have node_ to use, register the service client for this BT service
    getInput("service_name", service_name_);
    service_client_ = node_->create_client<ServiceT>(
      service_name_, rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

    // Make a request for the service without parameter
    request_ = std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    RCLCPP_DEBUG(
      node_->get_logger(), "[BTServiceNode][%s] Waiting for \"%s\" service", __func__,
      service_name_.c_str());
    if (!service_client_->wait_for_service(wait_for_service_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "[BTServiceNode][%s] \"%s\" service server not available after waiting for %.2fs", __func__,
        service_name_.c_str(), wait_for_service_timeout_.count() / 1000.0);
      throw std::runtime_error(
        std::string("Service server %s not available", service_node_name.c_str()));
    }

    RCLCPP_DEBUG(
      node_->get_logger(), "[BTServiceNode][%s] \"%s\" BtServiceNode initialized", __func__,
      service_node_name_.c_str());
  }

  // C++11的語法，意思是明確「刪除」這個 default constructor (無參數建構模式)
  BtServiceNode() = delete;

  virtual ~BtServiceNode() {}

  /**
   * @brief Any subclass of BtServiceNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override
  {
    // check whether request send or not
    if (!request_sent_) {
      // reset the flag to send the request or not,
      // allowing the user the option to set it in on_tick
      should_send_request_ = true;

      // user defined callback, may modify "should_send_request_".
      on_tick();

      if (!should_send_request_) {
        return BT::NodeStatus::FAILURE;
      }

      // store service future result handle into future_result variable
      future_result_ = service_client_->async_send_request(request_).share();
      sent_time_ = node_->now();
      request_sent_ = true;
    }
    return check_future();
  }

  /**
   * @brief The other (optional) override required by a BT service.
   */
  void halt() override
  {
    request_sent_ = false;
    setStatus(BT::NodeStatus::IDLE);
  }

  // 真正在繼承這個 BtServiceNode 需要實作的business logic 是底下這三個
  // - on_tick()               // 填寫 request 內容（從 port / blackboard 讀值塞進 service request內），或者社 should_send_request_ = false 來跳過
  // - on_completion()         // 處理 service 的回應（檢查結果、寫回blackboard），回傳最終 SUCCESS / FAILURE
  // - on_wait_for_result()    // 等待回應期間每次 timeout 時做的事

  /**
   * @brief Function to perform some user-defined operation on tick
   * Fill in service request with information if necessary
   */
  virtual void on_tick() {}

  /**
   * @brief Function to perform some user-defined operation upon successful
   * completion of the service. Could put a value on the blackboard.
   * @param response can be used to get the result of the service call in the BT Node.
   * @return BT::NodeStatus Returns SUCCESS by default, user may override to return another value
   */
  virtual BT::NodeStatus on_completion(std::shared_ptr<typename ServiceT::Response> /*response*/)
  {
    return BT::NodeStatus::SUCCESS;
  }

  /**
   * @brief Function to perform some user-defined operation after a timeout waiting
   * for a result that hasn't been received yet
   */
  virtual void on_wait_for_result() {}

  /**
   * @brief Check the future and decide the status of BT
   * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE otherwise
   */
  virtual BT::NodeStatus check_future()
  {
    auto elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
    auto remaining = server_timeout_ - elapsed;

    if (remaining > std::chrono::milliseconds(0)) {
      auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;

      // 主動「驅動 executor 去處理事件，把 response 變成 ready 的 future」
      rclcpp::FutureReturnCode rc;
      rc = callback_group_executor_.spin_until_future_complete(future_result_, timeout);
      if (rc == rclcpp::FutureReturnCode::SUCCESS) {
        request_sent_ = false;
        BT::NodeStatus status = on_completion(future_result_.get());
        return status;
      }

      if (rc == rclcpp::FutureReturnCode::TIMEOUT) {
        on_wait_for_result();
        elapsed = (node_->now() - sent_time_).template to_chrono<std::chrono::milliseconds>();
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
      }
    }

    RCLCPP_WARN(
      node_->get_logger(), "[BTServiceNode][%s] Node timed out while executing service call to %s.",
      __func__, service_name_.c_str());
    request_sent_ = false;
    return BT::NodeStatus::FAILURE;
  }

protected:
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

  std::string service_name_, service_node_name_;
  typename std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;
  std::shared_ptr<typename ServiceT::Request> request_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds server_timeout_;

  // The timeout value for BT loop execution
  std::chrono::milliseconds max_timeout_;

  // The timeout value for waiting for a service to response
  std::chrono::milliseconds wait_for_service_timeout_;

  // To track the server response when a new request is sent
  std::shared_future<typename ServiceT::Response::SharedPtr> future_result_;
  bool request_sent_{false};
  rclcpp::Time sent_time_;

  // Can be set in on_tick or on_wait_for_result to indicate if a request should be sent.
  bool should_send_request_;
};
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BT_SERVICE_NODE_HPP_