#ifndef SYNCAI_TEST_ACTION__TEST_ACTION_SERVER_HPP_
#define SYNCAI_TEST_ACTION__TEST_ACTION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "syncai_test_action/action/countdown.hpp"
#include "syncai_util/simple_action_server.hpp"

namespace syncai_test_action
{
class TestActionServer : public rclcpp::Node
{
public:
  // Countdown：由 Countdown.action 自動產生的型別，內含 ::Goal / ::Result / ::Feedback。
  using Countdown = syncai_test_action::action::Countdown;

  // 把 SimpleActionServer 用 Countdown 實體化 (ActionT = Countdown)。
  using ActionServer = syncai_util::SimpleActionServer<Countdown>;

  explicit TestActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~TestActionServer() = default;

  /**
   * @brief Second-phase init: 建立 action server 並 activate。
   */
  void initialize();

protected:
  /**
   * @brief Action server 的 execute callback。目前是 stub，第 4~7 步再長出來。
   */
  void executeCountdown();

  /**
   * @brief 測試用：收到 Trigger 服務呼叫就對 action server 做 deactivate()。
   * 讓你能在 goal 執行中 / idle 時手動觸發，觀察 deactivate 的行為。
   */
  void handleDeactivate(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // CountDown action server instance。用 unique_ptr 管理生命週期，建構子裡先不建立，等 initialize() 再建立。
  std::unique_ptr<ActionServer> action_server_;

  // 測試用觸發 deactivate 的服務。
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivate_service_;
};

}  // namespace syncai_test_action

#endif  // SYNCAI_TEST_ACTION__TEST_ACTION_SERVER_HPP_
