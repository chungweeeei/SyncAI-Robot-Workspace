#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COUNTDOWN_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COUNTDOWN_ACTION_HPP_

#include <memory>
#include <string>

#include "syncai_behavior_tree/bt_action_node.hpp"
#include "syncai_test_action/action/countdown.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtActionNode that wraps
 * syncai_test_action::action::Countdown — 當 Countdown action 的 client。
 *
 * 這個 subclass 很薄：送 goal / 等 ack / 收 feedback / 拿 result / cancel
 * 都由 BtActionNode 基底處理，這裡只負責「填 goal」「處理 result/feedback」。
 */
class CountdownAction : public BtActionNode<syncai_test_action::action::Countdown>
{
public:
  CountdownAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 每次要送 goal 前被呼叫：把 XML 上的 port 值塞進 goal_。
   */
  void on_tick() override;

  /**
   * @brief 等 result 的期間、每個 tick 被呼叫一次：印出最新 feedback。
   */
  void on_wait_for_result(
    std::shared_ptr<const syncai_test_action::action::Countdown::Feedback> feedback) override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief 宣告這個 node 的 port：seconds 為輸入，success/message 為輸出。
   *        providedBasicPorts 會併入基底的 server_name / server_timeout。
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("seconds", "Number of seconds to count down"),
        BT::OutputPort<bool>("success", "Whether the countdown completed successfully"),
        BT::OutputPort<std::string>("message", "Result message from the action server"),
      });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__COUNTDOWN_ACTION_HPP_
