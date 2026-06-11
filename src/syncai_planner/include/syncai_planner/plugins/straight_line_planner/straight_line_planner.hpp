#ifndef SYNCAI_PLANNER__PLUGINS__STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define SYNCAI_PLANNER__PLUGINS__STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_nav_core/global_planner.hpp"

namespace syncai_planner
{

/**
 * @class StraightLinePlanner
 * @brief A global planner that connects start and goal with a straight line,
 * ignoring obstacles. Useful as a minimal reference implementation of the
 * syncai_nav_core::GlobalPlanner plugin interface.
 */
class StraightLinePlanner : public syncai_nav_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  void initialize(
    const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Creating a plan from start and goal poses
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("StraightLinePlanner")};

  // Global Costmap
  syncai_costmap_2d::Costmap2D * costmap_{nullptr};

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Distance between interpolated poses on the line (meters)
  double interpolation_resolution_;
};

}  // namespace syncai_planner

#endif  // SYNCAI_PLANNER__PLUGINS__STRAIGHT_LINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
