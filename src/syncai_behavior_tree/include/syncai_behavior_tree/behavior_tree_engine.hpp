#ifndef SYNCAI_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define SYNCAI_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"

namespace syncai_behavior_tree
{

/**
 * @enum syncai_behavior_tree::BtStatus
 * @brief An enum class representing BT execution status
 */
enum class BtStatus { SUCCEEDED, FAILED, CANCELED };

/**
 * @brief A class to create and handle behavior trees
 */
class BehaviorTreeEngine
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::BehaviorTreeEngine
   * @param plugin_libraries vector of BT plugin library names to load
   * @details Constructor - registers the plugin libraries with BT::BehaviorTreeFactory, building
   *          its string -> class unordered_map
   */
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);

  /**
   * @brief A destructor for syncai_behavior_tree::BehaviorTreeEngine
   */
  virtual ~BehaviorTreeEngine() {}

  /**
   * @brief Function to execute a BT at a specific rate
   * @param tree BT to execute
   * @param onLoop Function to execute on each iteration of BT execution
   * @param cancelRequested Function to check if cancel was requested during BT execution
   * @param loopTimeout Time period for each iteration of BT execution
   * @return syncai_behavior_tree::BtStatus Status of BT execution
   * @details run() - loop that ticks the whole tree at a fixed rate until the tree returns
   *          SUCCESS/FAILURE or cancelRequested() returns true.
   */
  BtStatus run(
    BT::Tree * tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief Function to create a BT from a XML string
   * @param xml_string XML string representing BT
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromText(const std::string & xml_string, BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   * @param file_path Path to BT XML file
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromFile(const std::string & file_path, BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to explicitly reset all BT nodes to initial state
   * @param root_node Pointer to BT root node
   * @details Resets the whole tree to its initial state so it can be run again
   */
  void haltAllActions(BT::TreeNode * root_node);

protected:
  /**
    * The BT structure is written in an XML document, e.g.:
    * <Sequence>
    *   <ComputePathToPose/>
    *   <FollowPath/>
    * </Sequence>
    * 
    * The XML above carries only string names, so the program must know which C++ class
    * "ComputePathToPose" or "FollowPath" maps to before it can new an instance and place it
    * in the tree. BT::BehaviorTreeFactory is essentially the object that maintains that
    * "string -> class constructor" lookup table.
    */
  BT::BehaviorTreeFactory factory_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
