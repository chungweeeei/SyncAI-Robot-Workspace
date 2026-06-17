#include "syncai_behavior_tree/plugins/control/pipeline_sequence.hpp"

#include <sstream>
#include <stdexcept>
#include <string>

namespace syncai_behavior_tree
{

PipelineSequence::PipelineSequence(const std::string & name) : BT::ControlNode(name, {})
{
}

PipelineSequence::PipelineSequence(const std::string & name, const BT::NodeConfiguration & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus PipelineSequence::tick()
{
  /**
    * 以 planner / follower 套進去跑
    * children: index 0 = RateController(ComputePath)、 index 1 = FollowPath
    * 第一拍
    * - i = 0 RateController -> SUCCESS -> break，繼續
    * - i = 1 FollowPath -> RUNNING -> 判斷 1 >= 0 成立 -> last_child_ticked_ = 1、 return RUNNING
    * 
    * PipelineSequence的後續拍（未滿1s）
    * - i = 0 RateController -> RUNNING(節流，不算 path) -> 判斷 0 >= 1？不成立 -> break -> 不 return，繼續迴圈
    * - i = 1 FollowPath → RUNNING → :33 判斷 1 >= 1 成立 → return RUNNING
    */

  for (std::size_t i = 0; i < children_nodes_.size(); ++i) {
    auto status = children_nodes_[i]->executeTick();
    switch (status) {
      case BT::NodeStatus::FAILURE:
        ControlNode::haltChildren();
        last_child_ticked_ = 0;  // reset
        return status;
      case BT::NodeStatus::SUCCESS:
        // do nothing and continue on to the next child. If it is the last child
        // we'll exit the loop and hit the wrap-up code at the end of the method.
        break;
      case BT::NodeStatus::RUNNING:
        if (i >= last_child_ticked_) {
          last_child_ticked_ = i;
          return status;
        }
        // else do nothing and continue on to the next child
        // 前面的 child 在 RUNNING -> 不停，繼續迴圈。
        break;
      default:
        std::stringstream error_msg;
        error_msg << "Invalid node status. Received status " << status << "from child "
                  << children_nodes_[i]->name();
        throw std::runtime_error(error_msg.str());
    }
  }
  // Wrap up.
  ControlNode::haltChildren();
  last_child_ticked_ = 0;  // reset
  return BT::NodeStatus::SUCCESS;
}

void PipelineSequence::halt()
{
  BT::ControlNode::halt();
  last_child_ticked_ = 0;
}

}  // namespace syncai_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::PipelineSequence>("PipelineSequence");
}
