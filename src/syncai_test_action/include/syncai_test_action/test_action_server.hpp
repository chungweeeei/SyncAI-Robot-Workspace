#ifndef SYNCAI_TEST_ACTION__TEST_ACTION_SERVER_HPP_
#define SYNCAI_TEST_ACTION__TEST_ACTION_SERVER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_test_action/action/countdown.hpp"
#include "syncai_util/simple_action_server.hpp"

namespace syncai_test_action
{

/**
 * @class syncai_test_action::TestActionServer
 * @brief A minimal node hosting a syncai_util::SimpleActionServer on the
 * Countdown action, used to exercise and observe the server's goal
 * execution, feedback, cancellation and preemption logic.
 *
 * Follows the same two-phase init pattern as syncai_planner::PlannerServer:
 * the constructor only creates the node, and configure() — which must be
 * called after the node is owned by a shared_ptr — creates and activates the
 * action server (the SimpleActionServer constructor needs the node's
 * interfaces via shared_from_this()).
 */
class TestActionServer : public rclcpp::Node
{
public:
  using Countdown = syncai_test_action::action::Countdown;
  using ActionServer = syncai_util::SimpleActionServer<Countdown>;

  explicit TestActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~TestActionServer();

  /**
   * @brief Second-phase init: creates the action server on its own spin
   * thread and activates it.
   */
  void initialize();

protected:
  /**
   * @brief The action server execute callback: ticks down once per second,
   * publishing feedback, honoring cancel requests and accepting preempting
   * goals along the way.
   */
  void executeCountdown();

  std::unique_ptr<ActionServer> action_server_;
};

}  // namespace syncai_test_action

#endif  // SYNCAI_TEST_ACTION__TEST_ACTION_SERVER_HPP_
