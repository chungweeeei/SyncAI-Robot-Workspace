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
 * to prevent planning or control in restricted areas
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

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  std::unique_ptr<Costmap2D> mask_costmap_;

  std::string mask_frame_;    // Frame where mask located in
  std::string global_frame_;  // Frame of current layer (master_grid)

  unsigned int x_{0};
  unsigned int y_{0};
  unsigned int width_{0};
  unsigned int height_{0};
  bool has_updated_data_{false};
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__KEEPOUT_FILTER_HPP_
