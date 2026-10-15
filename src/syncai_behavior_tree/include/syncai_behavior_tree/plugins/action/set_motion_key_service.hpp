#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__SET_MOTION_KEY_SERVICE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__SET_MOTION_KEY_SERVICE_HPP_

#include <memory>
#include <string>

#include "syncai_behavior_tree/bt_service_node.hpp"
#include "syncai_common/srv/set_motion_key.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtServiceNode class that wraps
 * syncai_common::srv::SetMotionKey -- syncai_driver_manager's gait-mode
 * switch (set_motion_key). The key goes to the gait controller verbatim:
 * "0" stand, "1" RL locomotion, "2" lie down, "3" damping, "4" e-stop,
 * "5" MPC. The docking trees use it to switch to MPC before the approach and
 * back to RL afterwards.
 *
 * The response's `success` means the driver sent the UDP datagram, nothing
 * more: the gait controller never acknowledges a mode command, and the mode
 * it reports back (the `mode` topic) has no known value for MPC yet. The
 * tree is responsible for the wait that follows (dock.xml's header).
 */
class SetMotionKeyService : public BtServiceNode<syncai_common::srv::SetMotionKey>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::SetMotionKeyService
   * @param service_node_name BT node name
   * @param conf BT node configuration
   */
  SetMotionKeyService(const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief Fill the request from the `key` port; refuse to send without one
   */
  void on_tick() override;

  /**
   * @brief FAILURE with the driver's message on the blackboard when it
   * refused the key (LOCKED, unknown key)
   */
  BT::NodeStatus on_completion(
    std::shared_ptr<syncai_common::srv::SetMotionKey::Response> response) override;

  /**
   * @brief A driver that never answers says nothing about docking: report the
   * failure unattributed, so the navigator reports UNKNOWN with the message
   */
  void on_timeout() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    // No default on purpose: a tree must say which gait it wants.
    return providedBasicPorts({
      BT::InputPort<std::string>(
        "key",
        "Motion key, forwarded verbatim: 0 stand, 1 RL locomotion, 2 lie down, 3 damping, "
        "4 e-stop, 5 MPC"),
    });
  }

private:
  std::string key_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__SET_MOTION_KEY_SERVICE_HPP_
