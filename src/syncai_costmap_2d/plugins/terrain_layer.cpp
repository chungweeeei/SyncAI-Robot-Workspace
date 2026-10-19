#include "syncai_costmap_2d/terrain_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "syncai_costmap_2d/cost_values.hpp"
#include "syncai_util/path_height.hpp"
#include "syncai_util/stair_mode.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace syncai_costmap_2d
{

void TerrainLayer::onInitialize()
{
  if (!node_) {
    throw std::runtime_error{"Failed to get node in TerrainLayer"};
  }
  bool track_unknown_space = false;
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("pointlio/body_cloud")));
  declareParameter("sensor_height", rclcpp::ParameterValue(sensor_height_));
  declareParameter("max_range", rclcpp::ParameterValue(max_range_));
  declareParameter("min_rel_z", rclcpp::ParameterValue(min_rel_z_));
  declareParameter("max_rel_z", rclcpp::ParameterValue(max_rel_z_));
  declareParameter("plan_topic", rclcpp::ParameterValue(std::string("plan")));
  TerrainParams & p = terrain_.params;
  declareParameter("max_samples_per_cell", rclcpp::ParameterValue(p.max_samples_per_cell));
  declareParameter("ground_memory", rclcpp::ParameterValue(p.ground_memory));
  declareParameter("obstacle_memory", rclcpp::ParameterValue(p.obstacle_memory));
  declareParameter("min_ground_samples", rclcpp::ParameterValue(p.min_ground_samples));
  declareParameter("ground_quantile", rclcpp::ParameterValue(p.ground_quantile));
  declareParameter("max_ground_lift", rclcpp::ParameterValue(p.max_ground_lift));
  declareParameter("robot_radius", rclcpp::ParameterValue(p.robot_radius));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(p.min_obstacle_height));
  declareParameter("body_clearance", rclcpp::ParameterValue(p.body_clearance));
  declareParameter("max_step_height", rclcpp::ParameterValue(p.max_step_height));
  declareParameter("max_slope_deg", rclcpp::ParameterValue(p.max_slope_deg));
  declareParameter("slope_min_known", rclcpp::ParameterValue(p.slope_min_known));
  declareParameter("stair_min_rise", rclcpp::ParameterValue(p.stair_min_rise));
  declareParameter("stair_window", rclcpp::ParameterValue(p.stair_window));
  declareParameter(
    "stair_corridor_half_width", rclcpp::ParameterValue(p.stair_corridor_half_width));
  declareParameter("stair_obstacle_height", rclcpp::ParameterValue(p.stair_obstacle_height));

  node_->get_parameter(name_ + ".enabled", enabled_);
  node_->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
  node_->get_parameter(name_ + ".topic", cloud_topic_);
  node_->get_parameter(name_ + ".sensor_height", sensor_height_);
  node_->get_parameter(name_ + ".max_range", max_range_);
  node_->get_parameter(name_ + ".min_rel_z", min_rel_z_);
  node_->get_parameter(name_ + ".max_rel_z", max_rel_z_);
  node_->get_parameter(name_ + ".plan_topic", plan_topic_);
  node_->get_parameter(name_ + ".max_samples_per_cell", p.max_samples_per_cell);
  node_->get_parameter(name_ + ".ground_memory", p.ground_memory);
  node_->get_parameter(name_ + ".obstacle_memory", p.obstacle_memory);
  node_->get_parameter(name_ + ".min_ground_samples", p.min_ground_samples);
  node_->get_parameter(name_ + ".ground_quantile", p.ground_quantile);
  node_->get_parameter(name_ + ".max_ground_lift", p.max_ground_lift);
  node_->get_parameter(name_ + ".robot_radius", p.robot_radius);
  node_->get_parameter(name_ + ".min_obstacle_height", p.min_obstacle_height);
  node_->get_parameter(name_ + ".body_clearance", p.body_clearance);
  node_->get_parameter(name_ + ".max_step_height", p.max_step_height);
  node_->get_parameter(name_ + ".max_slope_deg", p.max_slope_deg);
  node_->get_parameter(name_ + ".slope_min_known", p.slope_min_known);
  node_->get_parameter(name_ + ".stair_min_rise", p.stair_min_rise);
  node_->get_parameter(name_ + ".stair_window", p.stair_window);
  node_->get_parameter(name_ + ".stair_corridor_half_width", p.stair_corridor_half_width);
  node_->get_parameter(name_ + ".stair_obstacle_height", p.stair_obstacle_height);
  node_->get_parameter("track_unknown_space", track_unknown_space);
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  // The costmap node lives in its own sub-namespace; both inputs are the
  // robot's (/<robot_id>/pointlio/body_cloud, /<robot_id>/plan).
  cloud_topic_ = joinWithParentNamespace(cloud_topic_);
  plan_topic_ = joinWithParentNamespace(plan_topic_);

  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = track_unknown_space ? NO_INFORMATION : FREE_SPACE;
  global_frame_ = layered_costmap_->getGlobalFrameID();
  TerrainLayer::matchSize();
  current_ = true;
  was_reset_ = false;
  last_stats_log_ = clock_->now();

  RCLCPP_INFO(
    logger_,
    "TerrainLayer %s: cloud %s (ground = lidar z - %.3f, returns within %.1f m and "
    "[%.1f, %.1f] m of the ground), path %s; step %.2f m, slope %.0f deg, obstacles %.2f-%.2f m "
    "above the ground (%.2f m on a climbing path), ground memory %.0f s, obstacle memory %.1f s",
    name_.c_str(), cloud_topic_.c_str(), sensor_height_, max_range_, min_rel_z_, max_rel_z_,
    plan_topic_.c_str(), p.max_step_height, p.max_slope_deg, p.min_obstacle_height,
    p.body_clearance, p.stair_obstacle_height, p.ground_memory, p.obstacle_memory);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
  custom_qos_profile.depth = 50;
  cloud_sub_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp::Node>>(
      node_, cloud_topic_, custom_qos_profile, sub_opt);
  cloud_sub_->unsubscribe();
  // The filter holds a cloud until its frame resolves in the costmap's frame
  // (odom <- map <- pointlio_odom <- pointlio_body: only after relocalize).
  cloud_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
    *cloud_sub_, *tf_, global_frame_, 50, node_->get_node_logging_interface(),
    node_->get_node_clock_interface(), tf2::durationFromSec(transform_tolerance_));
  cloud_filter_->registerCallback(
    std::bind(&TerrainLayer::cloudCallback, this, std::placeholders::_1));
  plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    plan_topic_, rclcpp::QoS(1),
    std::bind(&TerrainLayer::planCallback, this, std::placeholders::_1), sub_opt);
  cloud_sub_->subscribe();
}

void TerrainLayer::activate()
{
  if (cloud_sub_) {
    cloud_sub_->subscribe();
  }
}

void TerrainLayer::deactivate()
{
  if (cloud_sub_) {
    cloud_sub_->unsubscribe();
  }
}

void TerrainLayer::matchSize()
{
  CostmapLayer::matchSize();
  std::lock_guard<std::mutex> lock(terrain_mutex_);
  terrain_.resize(
    static_cast<int>(getSizeInCellsX()), static_cast<int>(getSizeInCellsY()), getResolution(),
    getOriginX(), getOriginY());
}

void TerrainLayer::reset()
{
  // A costmap clear (the BT's FollowPath retry) drops the costs, not the
  // samples: the floor the lidar can no longer see is in them, and a stale
  // obstacle mark leaves on its own after obstacle_memory. The next
  // updateBounds() rebuilds the costs from the same samples.
  resetMaps();
  current_ = false;
  was_reset_ = true;
}

void TerrainLayer::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(
      global_frame_, message->header.frame_id, message->header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "TerrainLayer: cannot transform %s into %s: %s",
      message->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return;
  }
  sensor_msgs::msg::PointCloud2 cloud;
  tf2::doTransform(*message, cloud, tf);

  const double ox = tf.transform.translation.x;
  const double oy = tf.transform.translation.y;
  // The ground under the robot: the lidar's origin minus its height above
  // the feet (the obstacle layer's height_reference: sensor, same number).
  const double ground_z = tf.transform.translation.z - sensor_height_;
  const double t = rclcpp::Time(message->header.stamp).seconds();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
  std::lock_guard<std::mutex> lock(terrain_mutex_);
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double x = *iter_x, y = *iter_y, z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (std::hypot(x - ox, y - oy) > max_range_) {
      continue;
    }
    const double rel = z - ground_z;
    if (rel < min_rel_z_ || rel > max_rel_z_) {
      continue;
    }
    terrain_.addSample(x, y, z, t);
  }
  robot_ground_z_ = ground_z;
  has_cloud_ = true;
}

void TerrainLayer::planCallback(nav_msgs::msg::Path::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  plan_ = *message;
}

std::vector<TerrainPathPose> TerrainLayer::climbingPoses()
{
  nav_msgs::msg::Path plan;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan = plan_;
  }
  std::vector<TerrainPathPose> out;
  // A level path (every 2D planner, and the empty one a cancel leaves)
  // climbs nothing, so a 2D map keeps every stair lethal.
  if (!syncai_util::pathHasHeight(plan)) {
    return out;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(global_frame_, plan.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "TerrainLayer: cannot transform the plan (%s) into %s: %s",
      plan.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
    return out;
  }
  const TerrainParams & p = terrain_.params;
  const double half = 0.5 * p.stair_window;
  for (size_t i = 0; i < plan.poses.size(); ++i) {
    if (syncai_util::StairMode::stairRise(plan, i, half, half) < p.stair_min_rise) {
      continue;
    }
    geometry_msgs::msg::PoseStamped in = plan.poses[i], pose;
    in.header = plan.header;
    tf2::doTransform(in, pose, tf);
    out.push_back({pose.pose.position.x, pose.pose.position.y, pose.pose.position.z});
  }
  return out;
}

void TerrainLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  std::lock_guard<std::mutex> lock(terrain_mutex_);
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    // After the costmap's own whole-cell rounding, so the two stay aligned.
    terrain_.updateOrigin(getOriginX(), getOriginY());
  }
  if (!enabled_) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  terrain_.setClimbingPoses(climbingPoses());
  terrain_.classify(clock_->now().seconds(), robot_ground_z_, costmap_, &last_stats_);

  // Every cell may have changed (a sample ages out, the window moves): the
  // whole layer is the update window, as the rolling static layer's is.
  *min_x = std::min(*min_x, getOriginX());
  *min_y = std::min(*min_y, getOriginY());
  *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
  *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());

  const auto now = clock_->now();
  if (now - last_stats_log_ > rclcpp::Duration::from_seconds(30.0)) {
    last_stats_log_ = now;
    RCLCPP_DEBUG(
      logger_,
      "TerrainLayer: %zu cells with a ground estimate, %zu lethal (%zu obstacle, %zu step, "
      "%zu slope), %zu on a stair corridor%s",
      last_stats_.known,
      last_stats_.lethal_obstacle + last_stats_.lethal_step + last_stats_.lethal_slope,
      last_stats_.lethal_obstacle, last_stats_.lethal_step, last_stats_.lethal_slope,
      last_stats_.corridor, has_cloud_ ? "" : " (no cloud yet)");
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void TerrainLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
  double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }
  transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
  for (const auto & pt : transformed_footprint_) {
    touch(pt.x, pt.y, min_x, min_y, max_x, max_y);
  }
}

void TerrainLayer::updateCosts(Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (footprint_clearing_enabled_) {
    // The floor under the robot is never seen (and its legs are in the
    // cloud): the footprint is free, as the obstacle layer has it.
    setConvexPolygonCost(transformed_footprint_, FREE_SPACE);
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace syncai_costmap_2d

PLUGINLIB_EXPORT_CLASS(syncai_costmap_2d::TerrainLayer, syncai_costmap_2d::Layer)
