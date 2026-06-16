#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__ASKMODEL_ACTION_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__ASKMODEL_ACTION_HPP_

#include <memory>
#include <string>

#include "syncai_behavior_tree/bt_action_node.hpp"
#include "syncai_test_bt_action_msgs/action/askmodel.hpp"

namespace syncai_behavior_tree
{

class AskmodelAction : public BtActionNode<syncai_test_bt_action_msgs::action::Askmodel>
{
public:
  AskmodelAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief 每次要送 goal 前被呼叫：把 XML 上的 port 值塞進 goal_。
   */
  void on_tick() override;

  /**
   * @brief 等 result 的期間、每個 tick 被呼叫一次：收到 feedback 時印一行 log。
   */
  void on_wait_for_result(
    std::shared_ptr<const syncai_test_bt_action_msgs::action::Askmodel::Feedback> feedback)
    override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("question", "The question to ask the model"),
      BT::OutputPort<bool>("success", "Whether the model provided an answer successfully"),
      BT::OutputPort<std::string>("answer", "The answer provided by the model"),
    });
  };
};
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__ASKMODEL_ACTION_HPP_