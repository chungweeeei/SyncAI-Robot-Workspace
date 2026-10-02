#ifndef SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_
#define SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_filters/costmap_filter.hpp"

namespace syncai_costmap_2d
{

/**
 * @class KeepoutFilter
 * @brief Reads in a keepout mask and marks keepout regions in the map
 * to prevent planning or control in restricted areas.
 *
 * Unlike upstream nav2, the mask is inflated here, by the filter, with the
 * same cost curve the costmap's InflationLayer applies to walls. Filters run
 * after the layer stack (inflation included), so an upstream keepout lands as
 * a bare LETHAL cell with no INSCRIBED band and no gradient around it. The
 * planner collision-checks the robot's centre cell against INSCRIBED, so two
 * zones one free cell apart were a plannable corridor (map dp2f had a 0.25 m
 * gap between two zones the planner routed a 0.44 m wide robot through). The
 * documented workaround — draw the footprint margin into the mask — bakes the
 * robot's size into per-map data and gives a hard band with no gradient to
 * steer the cost-aware search away from the edge. Inflating in the filter
 * makes a zone behave exactly like a wall: a gap narrower than twice the
 * inscribed radius closes, and the mask means "the forbidden area", nothing
 * more.
 */
class KeepoutFilter : public CostmapFilter
{
public:
  /**
   * @brief A constructor
   */
  KeepoutFilter();

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(const std::string & filter_info_topic) override;

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;

  /**
   * @brief Process the keepout layer at the current pose / bounds / grid
   */
  void process(
    Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose) override;

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter() override;

  /**
   * @brief If this filter is active
   */
  bool isActive();

  /**
   * @brief Re-inflate the mask for the new footprint. Called by
   * LayeredCostmap::setFootprint() on every filter; the inscribed radius the
   * inflation band is built from comes from that footprint.
   */
  void onFootprintChanged() override;

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Rebuild inflated_mask_ from mask_costmap_ using the costmap's
   * current inscribed radius and InflationLayer parameters. Runs once per
   * mask or footprint change, never per update cycle. Caller holds the mutex.
   */
  void inflateMask();

  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  // The mask as received: LETHAL where a zone is drawn, FREE / NO_INFORMATION
  // elsewhere. Kept next to the inflated copy because process() needs to tell
  // a drawn cell from an inflated one — only drawn cells may overwrite an
  // unknown master cell.
  std::unique_ptr<Costmap2D> mask_costmap_;
  // mask_costmap_ with the inflation band and gradient applied; what
  // process() actually writes into the master grid.
  std::unique_ptr<Costmap2D> inflated_mask_;

  std::string mask_frame_;    // Frame where mask located in
  std::string global_frame_;  // Frame of current layer (master_grid)

  // Set when inflated_mask_ was rebuilt; the next updateBounds() grows the
  // update window to the whole mask so every cell is rewritten once.
  bool has_updated_data_{false};
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_
