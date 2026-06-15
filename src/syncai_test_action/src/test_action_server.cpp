#include "syncai_test_action/test_action_server.hpp"

#include <chrono>
#include <memory>

namespace syncai_test_action
{

// ROS Node constructor
TestActionServer::TestActionServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("test_action_server", options)
{
}

void TestActionServer::initialize()
{
  RCLCPP_INFO(this->get_logger(), "[TestActionServer][%s] Configuring", __func__);

  // 建立 SimpleActionServer
  action_server_ = std::make_unique<ActionServer>(
    this->shared_from_this(),                              // node：餵 node interfaces 用
    "countdown",                                           // action_name
    std::bind(&TestActionServer::executeCountdown, this),  // execute_callback
    nullptr,                                               // completion_callback
    // server_timeout => 系統要準備關掉這個 action server 了，但你的 executeCountdown 可能還卡在某個倒數迴圈內，這匙系統園藝給你多久的時間把自己收乾淨，超過這個時間後系統就會直接 goal abort 掉。
    std::chrono::milliseconds(1000),
    true);  // spin_thread：開專屬執行緒

  // 讓 server 開始接受 goal。
  action_server_->activate();
  RCLCPP_INFO(
    this->get_logger(), "[TestActionServer][%s] Countdown action server is active", __func__);

  // 測試用：~/deactivate 服務，呼叫即觸發 action_server_->deactivate()。
  deactivate_service_ = this->create_service<std_srvs::srv::Trigger>(
    "~/deactivate",
    std::bind(
      &TestActionServer::handleDeactivate, this, std::placeholders::_1, std::placeholders::_2));
}

void TestActionServer::handleDeactivate(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Deactivate requested via service");
  // 注意：deactivate() 會阻塞直到 worker 結束（或超過 server_timeout 強制終止），
  // 因此這個 service callback 也會被卡住直到 deactivate 回來。
  action_server_->deactivate();
  response->success = true;
  response->message = "action server deactivated";
}

void TestActionServer::executeCountdown()
{
  // 先拿到目前正在處理的 goal。
  auto goal = action_server_->get_current_goal();
  if (!goal) {
    // 沒有有效 goal（理論上不該發生）→ 把 current 終止掉，直接返回。
    action_server_->terminate_current();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal: count down from %d", goal->seconds);

  auto feedback = std::make_shared<Countdown::Feedback>();
  auto result = std::make_shared<Countdown::Result>();

  int remaining = goal->seconds;
  rclcpp::Rate loop_rate(1.0);  // 1 Hz：每圈大約 1 秒

  // 倒數迴圈：每秒檢查 deactivate/cancel/preempt、發一次 feedback，數到 0 為止。
  while (rclcpp::ok()) {
    // ⓪ server 被 deactivate → 主動收尾，讓 deactivate() 能在一個 tick 內放行。
    if (!action_server_->is_server_active()) {
      RCLCPP_INFO(this->get_logger(), "Server deactivated, stopping current goal");
      result->success = false;
      result->message = "deactivated";
      action_server_->terminate_all(result);
      return;
    }

    // ① client 要求取消 → 回傳失敗的 result 並終止。
    if (action_server_->is_cancel_requested()) {
      RCLCPP_INFO(this->get_logger(), "Goal canceled with %d seconds remaining", remaining);
      result->success = false;
      result->message = "canceled";
      action_server_->terminate_all(result);
      return;
    }

    // ② 有新 goal 在等（preempt）→ 接受它、切換到新 goal 繼續數。
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(this->get_logger(), "Preempting current goal with a new one");
      goal = action_server_->accept_pending_goal();
      remaining = goal->seconds;
      RCLCPP_INFO(this->get_logger(), "New goal: count down from %d", goal->seconds);
    }

    if (remaining <= 0) {
      break;
    }

    RCLCPP_INFO(this->get_logger(), "Counting down: %d", remaining);
    feedback->remaining_seconds = remaining;
    action_server_->publish_feedback(feedback);  // 推送進度給 client

    loop_rate.sleep();  // 睡到湊滿這一圈的 1 秒
    --remaining;
  }

  // 數到 0 → 標記成功並回傳。
  result->success = true;
  result->message = "testing ....";
  action_server_->succeeded_current(result);
  RCLCPP_INFO(this->get_logger(), "Countdown completed successfully");
}

}  // namespace syncai_test_action
