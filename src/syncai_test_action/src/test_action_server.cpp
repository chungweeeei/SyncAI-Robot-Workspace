#include "syncai_test_action/test_action_server.hpp"

#include <chrono>
#include <memory>

namespace syncai_test_action
{

// ROS Node constructor
TestActionServer::TestActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("test_action_server", options)
{
  RCLCPP_INFO(this->get_logger(), "[TestActionServer][%s] Creating", __func__);
}

TestActionServer::~TestActionServer()
{
  action_server_.reset();
}

void TestActionServer::initialize()
{
  RCLCPP_INFO(this->get_logger(), "[TestActionServer][%s] Configuring", __func__);

  /**
   * 在simple_action_server.hpp裡面定義的是：
   * typedef std::function<void()> ExecuteCallback -> 是一個「不帶參數、可以直接呼叫」的東西(可以直接想成是個 callback function)
   * SimpleActionServer的Constructor會需要一個ExecuteCallback的參數
   * 問題： member function不能直接傳進去
   *       - 不能只傳 &TestActionServer::executeCountdown，主要是因為member function有個隱藏參數 - this。
   *       - 在executeCountdown() 裡面用到的 action_server_、get_logger()，這些都是「某個特定物件」的成員，沒有物件實例根本無法執行。
   * std::bind(&TestActionServer::executeCountdown, this)
   *                  ↑ 要呼叫的成員函式                 ↑ 把這個物件實例「綁」進去
   * 
   */
  action_server_ = std::make_unique<ActionServer>(
    this->shared_from_this(), "countdown", std::bind(&TestActionServer::executeCountdown, this),
    nullptr, std::chrono::milliseconds(500), true);

  // Activate the action server so it can start accepting goals.
  action_server_->activate();
  RCLCPP_INFO(
    this->get_logger(), "[TestActionServer][%s] Countdown action server is active", __func__);
}

void TestActionServer::executeCountdown()
{
  auto goal = action_server_->get_current_goal();
  if (!goal) {
    action_server_->terminate_current();
    return;
  }

  auto feedback = std::make_shared<Countdown::Feedback>();
  auto result = std::make_shared<Countdown::Result>();

  RCLCPP_INFO(get_logger(), "Received goal: count down from %d", goal->seconds);

  int remaining = goal->seconds;
  rclcpp::Rate loop_rate(1.0);

  /**
   * 這裡的 while loop 主要負責：
   * 1. 每秒檢查 cancel / preempt 請求
   * 2. publish action feedback
   */
  while (rclcpp::ok()) {
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(get_logger(), "Goal canceled with %d seconds remaining", remaining);
      result->success = false;
      action_server_->terminate_all(result);
      return;
    }

    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(get_logger(), "Preempting current goal with a new one");
      goal = action_server_->accept_pending_goal();
      remaining = goal->seconds;
      RCLCPP_INFO(get_logger(), "New goal: count down from %d", goal->seconds);
    }

    if (remaining <= 0) {
      break;
    }

    RCLCPP_INFO(get_logger(), "Counting down: %d", remaining);
    feedback->remaining_seconds = remaining;
    action_server_->publish_feedback(feedback);

    loop_rate.sleep();
    --remaining;
  }

  result->success = true;
  action_server_->succeeded_current(result);
  RCLCPP_INFO(get_logger(), "Countdown completed successfully");
}

}  // namespace syncai_test_action
