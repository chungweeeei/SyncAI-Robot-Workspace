#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__PRINTOUT_SERVICE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__PRINTOUT_SERVICE_HPP_

#include <memory>
#include <string>

#include "syncai_behavior_tree/bt_service_node.hpp"
#include "syncai_test_bt_action_msgs/srv/printout.hpp"

namespace syncai_behavior_tree
{

class PrintoutService : public BtServiceNode<syncai_test_bt_action_msgs::srv::Printout>
{
public:
  PrintoutService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf,
    const std::string & service_name = "");

  /**
   * @brief 每次要送 request 前被呼叫：把 XML 上的 "text" port 值塞進 request_。
   */
  void on_tick() override;

  /**
   * @brief 收到 service response 時被呼叫：把 response->success 寫回 "success" output
   * port，並依其值決定 BT node 回傳 SUCCESS 或 FAILURE。
   */
  BT::NodeStatus on_completion(
    std::shared_ptr<syncai_test_bt_action_msgs::srv::Printout::Response> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("text", "The text to print out"),
      BT::OutputPort<bool>("success", "Whether the service printed the text successfully"),
    });
  }
};
}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__PRINTOUT_SERVICE_HPP_