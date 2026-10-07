#include "syncai_behavior_tree/plugins/control/recovery_node.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace syncai_behavior_tree
{
RecoveryNode::RecoveryNode(const std::string & name, const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0),
  retry_refill_time_(0.0)
{
  getInput("number_of_retries", number_of_retries_);
  getInput("retry_refill_time", retry_refill_time_);
}

BT::NodeStatus RecoveryNode::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException("Recovery Node '" + name() + "' must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    if (current_child_idx_ == 0 && child_node->status() == BT::NodeStatus::IDLE) {
      first_child_start_ = std::chrono::steady_clock::now();
    }
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS: {
          // reset node and return success when first child returns success
          halt();
          return BT::NodeStatus::SUCCESS;
        }

        case BT::NodeStatus::FAILURE: {
          // Upstream only resets retry_count_ in halt(), i.e. when this node
          // returns SUCCESS or FAILURE. Around a child 0 that stays RUNNING for
          // a whole goal (FollowPath) that makes number_of_retries a budget
          // per *goal*: a blocker met in the first minute used the retry up,
          // and one met ten minutes later failed the goal outright. An
          // attempt that ran at least retry_refill_time before failing earns
          // the budget back, so it is per *incident* instead. Time rather than
          // "child 0 returned RUNNING" because the failing attempt itself is
          // RUNNING for a while (FollowPath: failure_tolerance, or the
          // progress checker's movement_time_allowance) -- refilling on
          // RUNNING would retry a dead end forever. With retry_refill_time
          // longer than any attempt that never gets going, a stuck robot
          // still runs out of retries; only one that drove for long enough
          // between failures keeps getting them.
          if (
            retry_count_ > 0 && retry_refill_time_ > 0.0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - first_child_start_)
                .count() >= retry_refill_time_) {
            RCLCPP_INFO(
              rclcpp::get_logger("RecoveryNode"),
              "[%s] '%s' failed after running >= %.1f s; retry budget refilled (%u used)",
              __func__, name().c_str(), retry_refill_time_, retry_count_);
            retry_count_ = 0;
          }
          if (retry_count_ < number_of_retries_) {
            // halt first child and tick second child in next iteration
            ControlNode::haltChild(0);
            current_child_idx_++;
            break;
          } else {
            // reset node and return failure when max retries has been exceeded
            halt();
            return BT::NodeStatus::FAILURE;
          }
        }

        case BT::NodeStatus::RUNNING: {
          return BT::NodeStatus::RUNNING;
        }

        default: {
          throw BT::LogicError("A child node must never return IDLE");
        }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS: {
          // halt second child, increment recovery count, and tick first child in next iteration
          ControlNode::haltChild(1);
          retry_count_++;
          current_child_idx_--;
        } break;

        case BT::NodeStatus::FAILURE: {
          // reset node and return failure if second child fails
          halt();
          return BT::NodeStatus::FAILURE;
        }

        case BT::NodeStatus::RUNNING: {
          return BT::NodeStatus::RUNNING;
        }

        default: {
          throw BT::LogicError("A child node must never return IDLE");
        }
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNode::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<syncai_behavior_tree::RecoveryNode>("RecoveryNode");
}
