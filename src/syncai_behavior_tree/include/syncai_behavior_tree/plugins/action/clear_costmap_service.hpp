#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_

#include <string>

#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "syncai_behavior_tree/bt_service_node.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief A syncai_behavior_tree::BtServiceNode class that wraps
 * nav2_msgs::srv::ClearEntireCostmap. The costmap to clear is selected with
 * the service_name port, e.g. "global_costmap/clear_entirely_global_costmap".
 */
class ClearEntireCostmapService : public BtServiceNode<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::ClearEntireCostmapService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ClearEntireCostmapService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
};

/**
 * @brief A syncai_behavior_tree::BtServiceNode class that wraps
 * nav2_msgs::srv::ClearCostmapExceptRegion. Clears everything outside a
 * square of reset_distance around the robot.
 */
class ClearCostmapExceptRegionService
: public BtServiceNode<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::ClearCostmapExceptRegionService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ClearCostmapExceptRegionService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>(
        "reset_distance", 1, "Distance from the robot above which obstacles are cleared"),
    });
  }
};

/**
 * @brief A syncai_behavior_tree::BtServiceNode class that wraps
 * nav2_msgs::srv::ClearCostmapAroundRobot. Clears a square of reset_distance
 * around the robot.
 */
class ClearCostmapAroundRobotService : public BtServiceNode<nav2_msgs::srv::ClearCostmapAroundRobot>
{
public:
  /**
   * @brief A constructor for syncai_behavior_tree::ClearCostmapAroundRobotService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ClearCostmapAroundRobotService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>(
        "reset_distance", 1, "Distance from the robot under which obstacles are cleared"),
    });
  }
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_SERVICE_HPP_
