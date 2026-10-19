#ifndef SYNCAI_COSTMAP_2D__TERRAIN_LAYER_HPP_
#define SYNCAI_COSTMAP_2D__TERRAIN_LAYER_HPP_

// TerrainLayer (2026-10-20): the local costmap's obstacle source for a
// legged robot, in place of ObstacleLayer's height band. It feeds the lidar
// cloud and the global path into a TerrainGrid (terrain_grid.hpp has the
// design and the rules) and writes the grid's verdict -- LETHAL for a wall
// foot, a box, a person, a step the legs cannot take, a drop-off or a slope
// too steep; FREE for the floor, a kerb, a ramp and the stair the path
// climbs; NO_INFORMATION where nothing has been seen -- into its costmap,
// which the InflationLayer then inflates as it did the obstacle layer's.
// One layer for both map types: on a 2D map the path has no heights and
// every stair stays lethal; on a 3D map (OctomapPlanner3D) the path's stair
// links open a corridor through it.
//
// Not a second observation source beside the obstacle layer but its
// replacement in the local costmap: the two would disagree on a ramp (free
// here, a wall there), and the max of the two is the wall. The global
// costmap keeps the obstacle layer: its planner judges 2D maps on it with
// the static map underneath, and Octo3D reads no cells at all.

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "syncai_costmap_2d/costmap_layer.hpp"
#include "syncai_costmap_2d/footprint.hpp"
#include "syncai_costmap_2d/layered_costmap.hpp"
#include "syncai_costmap_2d/terrain_grid.hpp"

namespace syncai_costmap_2d
{

class TerrainLayer : public CostmapLayer
{
public:
  TerrainLayer() { costmap_ = nullptr; }
  virtual ~TerrainLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }
  void matchSize() override;
  void activate() override;
  void deactivate() override;

protected:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void planCallback(nav_msgs::msg::Path::ConstSharedPtr message);
  // The climbing stretches of the latest path, in the costmap's frame.
  std::vector<TerrainPathPose> climbingPoses();
  void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y);

  std::string global_frame_;
  bool rolling_window_{true};
  bool was_reset_{false};
  bool footprint_clearing_enabled_{true};
  std::vector<geometry_msgs::msg::Point> transformed_footprint_;

  // The cloud: its frame is the lidar's, whose origin minus sensor_height is
  // the ground under the robot (the obstacle layer's height_reference:
  // sensor). Returns farther than max_range in the plane, or outside
  // [min_rel_z, max_rel_z] of that ground, are dropped before the grid.
  std::string cloud_topic_;
  double sensor_height_{0.481};
  double max_range_{3.0};
  double min_rel_z_{-1.5};
  double max_rel_z_{1.5};
  double transform_tolerance_{0.3};
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp::Node>>
    cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> cloud_filter_;

  // The path: the planner's `plan` (republished at plan_republish_rate with
  // the heights OctomapPlanner3D wrote), read for its climbing stretches.
  std::string plan_topic_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  std::mutex plan_mutex_;
  nav_msgs::msg::Path plan_;

  // The grid and what the last cloud said about the robot. terrain_mutex_
  // orders the subscription callbacks against the costmap update thread.
  std::mutex terrain_mutex_;
  TerrainGrid terrain_;
  bool has_cloud_{false};
  double robot_ground_z_{0.0};
  TerrainStats last_stats_;
  rclcpp::Time last_stats_log_;
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__TERRAIN_LAYER_HPP_
