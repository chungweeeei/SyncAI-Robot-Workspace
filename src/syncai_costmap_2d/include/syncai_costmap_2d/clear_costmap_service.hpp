#ifndef SYNCAI_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_
#define SYNCAI_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_layer.hpp"

namespace syncai_costmap_2d
{
class Costmap2DROS;

/**
 * @class ClearCostmapService
 * @brief Exposes services to clear costmap objects in inclusive/exclusive regions or completely
 */
class ClearCostmapService
{
public:
  /**
   * @brief A constructor. Creates the clear costmap services on the given node.
   */
  ClearCostmapService(const rclcpp::Node::SharedPtr & node, Costmap2DROS & costmap);

  /**
   * @brief A destructor. Shuts the services down.
   */
  ~ClearCostmapService();

  // The ClearCostmapService is not copyable and requires a node/costmap.
  ClearCostmapService() = delete;

  /**
   * @brief Clears the region set by a distance around the robot. If invert, clears
   * all but that region.
   */
  void clearRegion(const double reset_distance, bool invert);

  /**
   * @brief Clears all layers of the costmap entirely.
   */
  void clearEntirely();

private:
  // The Logger object for logging
  rclcpp::Logger logger_{rclcpp::get_logger("syncai_costmap_2d")};

  // The costmap to clear
  Costmap2DROS & costmap_;

  // Clearing parameters
  unsigned char reset_value_;

  // Server for clearing the costmap
  rclcpp::Service<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr clear_except_service_;

  /**
   * @brief Callback to clear costmap except in a given region
   */
  void clearExceptRegionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_around_service_;
  /**
   * @brief Callback to clear costmap in a given region
   */
  void clearAroundRobotCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response);

  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_entire_service_;
  /**
   * @brief Callback to clear costmap
   */
  void clearEntireCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response);

  /**
   * @brief  Function used to clear a given costmap layer
   */
  void clearLayerRegion(
    std::shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
    bool invert);

  /**
   * @brief Get the robot's position in the costmap using the master costmap
   */
  bool getPosition(double & x, double & y) const;
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__CLEAR_COSTMAP_SERVICE_HPP_