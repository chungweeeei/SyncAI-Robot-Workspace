#ifndef SYNCAI_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_
#define SYNCAI_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d.hpp"
#include "syncai_util/robot_utils.hpp"

namespace syncai_costmap_2d
{
typedef std::vector<geometry_msgs::msg::Point> Footprint;

/**
 * @class FootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */
template <typename CostmapT>
class FootprintCollisionChecker
{
public:
  /**
   * @brief A constructor.
   */
  FootprintCollisionChecker();
  /**
   * @brief A constructor.
   */
  explicit FootprintCollisionChecker(CostmapT costmap);
  /**
   * @brief Find the footprint cost in oriented footprint
   */
  double footprintCost(const Footprint footprint);
  /**
   * @brief Find the footprint cost a a post with an unoriented footprint
   */
  double footprintCostAtPose(double x, double y, double theta, const Footprint footprint);
  /**
   * @brief Get the cost for a line segment
   */
  double lineCost(int x0, int x1, int y0, int y1) const;
  /**
   * @brief Get the map coordinates from a world point
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  /**
   * @brief Get the cost of a point
   */
  double pointCost(int x, int y) const;
  /**
  * @brief Set the current costmap object to use for collision detection
  */
  void setCostmap(CostmapT costmap);
  /**
  * @brief Get the current costmap object
  */
  CostmapT getCostmap() { return costmap_; }

protected:
  CostmapT costmap_;
};
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_