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
 * @class syncai_behavior_tree::BehaviorTreeEngine
 * @brief A class to create and handle behavior trees
 */
class BehaviorTreeEngine
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::BehaviorTreeEngine
   * @param plugin_libraries vector of BT plugin library names to load
   */
  explicit BehaviorTreeEngine(const std::vector<std::string> & plugin_libraries);
  virtual ~BehaviorTreeEngine() {}

  /**
   * @brief Function to execute a BT at a specific rate
   * @param tree BT to execute
   * @param onLoop Function to execute on each iteration of BT execution
   * @param cancelRequested Function to check if cancel was requested during BT execution
   * @param loopTimeout Time period for each iteration of BT execution
   * @return syncai_behavior_tree::BtStatus Status of BT execution
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
   */
  void haltAllActions(BT::TreeNode * root_node);

protected:
  /**
    * BT Tree的結構是寫在一份 XML 文件裡，例如：
    * <Sequence>
    *   <ComputePathToPose/>
    *   <FollowPath/>
    * </Sequence>
    * 
    * 從上面的 XML 文件裡只有字串名稱，程式裡必須知道 "ComputePathToPose" 或者 "FollowPath" 對應到哪個 C++ class.
    * 這樣才可以 new 一個class instance 出來放在BT Tree裡。
    * 而BT::BehaviorTreeFactory主要就是維護「string -> class constructor」對照表的物件。
    */
  BT::BehaviorTreeFactory factory_;
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
