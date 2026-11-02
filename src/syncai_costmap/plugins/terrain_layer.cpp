#include "syncai_costmap/terrain_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "syncai_costmap/cost_values.hpp"
#include "syncai_util/path_height.hpp"
#include "syncai_util/plan_crossings.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace syncai_costmap
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
  declareParameter("cloud_timeout", rclcpp::ParameterValue(cloud_timeout_));
  TerrainParams & p = terrain_.params;
  declareParameter("max_samples_per_cell", rclcpp::ParameterValue(p.max_samples_per_cell));
  declareParameter("ground_memory", rclcpp::ParameterValue(p.ground_memory));
  declareParameter("obstacle_memory", rclcpp::ParameterValue(p.obstacle_memory));
  declareParameter("ray_clear_tolerance", rclcpp::ParameterValue(p.ray_clear_tolerance));
  declareParameter("ray_clear_end_margin", rclcpp::ParameterValue(p.ray_clear_end_margin));
  declareParameter("min_ground_samples", rclcpp::ParameterValue(p.min_ground_samples));
  declareParameter("ground_quantile", rclcpp::ParameterValue(p.ground_quantile));
  declareParameter("max_ground_lift", rclcpp::ParameterValue(p.max_ground_lift));
  declareParameter("sparse_ground_radius", rclcpp::ParameterValue(p.sparse_ground_radius));
  declareParameter("robot_radius", rclcpp::ParameterValue(p.robot_radius));
  declareParameter("min_obstacle_height", rclcpp::ParameterValue(p.min_obstacle_height));
  declareParameter("body_clearance", rclcpp::ParameterValue(p.body_clearance));
  declareParameter("max_step_height", rclcpp::ParameterValue(p.max_step_height));
  declareParameter("max_slope_deg", rclcpp::ParameterValue(p.max_slope_deg));
  declareParameter("slope_min_known", rclcpp::ParameterValue(p.slope_min_known));
  declareParameter("crossing_radius", rclcpp::ParameterValue(p.crossing_radius));
  declareParameter("crossing_z_band", rclcpp::ParameterValue(p.crossing_z_band));

  node_->get_parameter(name_ + ".enabled", enabled_);
  node_->get_parameter(name_ + ".footprint_clearing_enabled", footprint_clearing_enabled_);
  node_->get_parameter(name_ + ".topic", cloud_topic_);
  node_->get_parameter(name_ + ".sensor_height", sensor_height_);
  node_->get_parameter(name_ + ".max_range", max_range_);
  node_->get_parameter(name_ + ".min_rel_z", min_rel_z_);
  node_->get_parameter(name_ + ".max_rel_z", max_rel_z_);
  node_->get_parameter(name_ + ".plan_topic", plan_topic_);
  node_->get_parameter(name_ + ".cloud_timeout", cloud_timeout_);
  node_->get_parameter(name_ + ".max_samples_per_cell", p.max_samples_per_cell);
  node_->get_parameter(name_ + ".ground_memory", p.ground_memory);
  node_->get_parameter(name_ + ".obstacle_memory", p.obstacle_memory);
  node_->get_parameter(name_ + ".ray_clear_tolerance", p.ray_clear_tolerance);
  node_->get_parameter(name_ + ".ray_clear_end_margin", p.ray_clear_end_margin);
  node_->get_parameter(name_ + ".min_ground_samples", p.min_ground_samples);
  node_->get_parameter(name_ + ".ground_quantile", p.ground_quantile);
  node_->get_parameter(name_ + ".max_ground_lift", p.max_ground_lift);
  node_->get_parameter(name_ + ".sparse_ground_radius", p.sparse_ground_radius);
  node_->get_parameter(name_ + ".robot_radius", p.robot_radius);
  node_->get_parameter(name_ + ".min_obstacle_height", p.min_obstacle_height);
  node_->get_parameter(name_ + ".body_clearance", p.body_clearance);
  node_->get_parameter(name_ + ".max_step_height", p.max_step_height);
  node_->get_parameter(name_ + ".max_slope_deg", p.max_slope_deg);
  node_->get_parameter(name_ + ".slope_min_known", p.slope_min_known);
  node_->get_parameter(name_ + ".crossing_radius", p.crossing_radius);
  node_->get_parameter(name_ + ".crossing_z_band", p.crossing_z_band);
  node_->get_parameter("track_unknown_space", track_unknown_space);
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  // The costmap node lives in its own sub-namespace; every input is the
  // robot's (/<robot_id>/pointlio/body_cloud, /<robot_id>/plan). It runs on
  // every map: until 2026-10-28 a MapTypeGate ran it on a 3D map only, with
  // the obstacle layer on a 2D one, and both went with 2D navigation.
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
    "above the ground, ground memory %.0f s, obstacle memory %.1f s, ray clearing %s "
    "(tolerance %.2f m, end margin %.2f m); not current once the newest cloud is older than "
    "%.1f s (cloud_timeout; 0 = never)",
    name_.c_str(), cloud_topic_.c_str(), sensor_height_, max_range_, min_rel_z_, max_rel_z_,
    plan_topic_.c_str(), p.max_step_height, p.max_slope_deg, p.min_obstacle_height,
    p.body_clearance, p.ground_memory, p.obstacle_memory,
    p.ray_clear_tolerance > 0.0 ? "on" : "off", p.ray_clear_tolerance, p.ray_clear_end_margin,
    cloud_timeout_);

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
  // Beside `plan`, the planner's (latched).
  crossings_sub_ = node_->create_subscription<std_msgs::msg::String>(
    joinWithParentNamespace(syncai_util::kPlanCrossingsTopic),
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&TerrainLayer::crossingsCallback, this, std::placeholders::_1), sub_opt);
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

  // The lidar's origin: the ray origin, and sensor_height under it the
  // ground under the robot (the obstacle layer's height_reference: sensor,
  // same number). One origin for the whole cloud is right because pointlio
  // deskews every cloud into the body frame at the end-of-scan stamp.
  const double ox = tf.transform.translation.x;
  const double oy = tf.transform.translation.y;
  const double oz = tf.transform.translation.z;
  const double ground_z = oz - sensor_height_;
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
    // Every return clears along its ray, the far ones (a wall at 6 m, the
    // ceiling) most usefully: the range and height filters below are for
    // what is stored, not for what was seen through.
    terrain_.clearRay(ox, oy, oz, x, y, z, t);
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
  // Newest, not last: the message filter can release a cloud held for its
  // TF after a later one.
  const int64_t ns = rclcpp::Time(message->header.stamp, clock_->get_clock_type()).nanoseconds();
  int64_t seen = newest_cloud_ns_.load();
  while (ns > seen && !newest_cloud_ns_.compare_exchange_weak(seen, ns)) {
  }
}

double TerrainLayer::cloudAge() const
{
  const int64_t ns = newest_cloud_ns_.load();
  if (ns == 0) {
    return -1.0;
  }
  return (clock_->now().nanoseconds() - ns) * 1e-9;
}

bool TerrainLayer::isCurrent() const
{
  // Until 2026-11 only current_, which a costmap clear drops for one update
  // and nothing else ever did: with pointlio/body_cloud gone (Point-LIO hung
  // or restarted, as on 2026-11-03 / -04) the samples aged out in 2-10 s,
  // every cell went NO_INFORMATION -- FREE in the master grid, since
  // track_unknown_space is off -- and MPPI drove on an empty costmap while
  // the controller's "costmap not current" wait never fired.
  //
  // The cloud's own stamp, not its arrival: the message filter drops a
  // cloud whose TF is missing (lio_bridge or the localizer down), and a
  // Point-LIO that has fallen behind (2026-10-28: ~30 s, then minutes)
  // still publishes, with stamps as old as its odometry. Both read stale.
  // The grid ages its samples against the same clock (updateBounds()'s
  // classify(now)), which is what makes the stamps comparable to it.
  if (!current_) {
    return false;
  }
  if (cloud_timeout_ <= 0.0) {
    return true;
  }
  const double age = cloudAge();
  return age >= 0.0 && age <= cloud_timeout_;
}

std::string TerrainLayer::staleReason() const
{
  if (!current_) {
    return "cleared, waiting for its next update";
  }
  if (cloud_timeout_ <= 0.0) {
    return "";
  }
  const double age = cloudAge();
  char buf[160];
  if (age < 0.0) {
    std::snprintf(
      buf, sizeof buf, "no cloud on %s yet (or none whose TF into %s resolved)",
      cloud_topic_.c_str(), global_frame_.c_str());
  } else if (age > cloud_timeout_) {
    std::snprintf(
      buf, sizeof buf, "newest cloud on %s is %.1f s old (cloud_timeout %.1f s)",
      cloud_topic_.c_str(), age, cloud_timeout_);
  } else {
    return "";
  }
  return buf;
}

void TerrainLayer::planCallback(nav_msgs::msg::Path::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(plan_mutex_);
  plan_ = *message;
}

void TerrainLayer::crossingsCallback(std_msgs::msg::String::ConstSharedPtr message)
{
  syncai_util::PlanCrossings c;
  std::string error;
  if (!syncai_util::fromJson(message->data, c, error)) {
    RCLCPP_WARN(
      logger_,
      "TerrainLayer: plan_crossings unreadable (%s): kept the last one, so its crossings stay "
      "waived until the next plan",
      error.c_str());
    return;
  }
  // known: false (a planner that says nothing) waives nothing, as no plan does.
  std::vector<std::vector<TerrainPathPose>> flat;
  if (c.known) {
    for (const auto & x : c.crossings) {
      if (x.stair_gait || x.pts.empty()) {
        continue;  // a stair stays lethal: the stair mode climbs it blind
      }
      if (x.kind == "narrow") {
        continue;  // a passage's width, not a sill or a slope the planner judged walkable
      }
      std::vector<TerrainPathPose> pts;
      pts.reserve(x.pts.size());
      for (const auto & p : x.pts) {
        pts.push_back({p.x, p.y, p.z});
      }
      flat.push_back(std::move(pts));
    }
  }
  if (!flat.empty()) {
    RCLCPP_INFO(
      logger_,
      "TerrainLayer: %zu flat-gait crossings on the new plan: their step and slope rules waived "
      "within %.2f m",
      flat.size(), terrain_.params.crossing_radius);
  }
  std::lock_guard<std::mutex> lock(plan_mutex_);
  crossings_frame_ = c.frame_id;
  crossings_ = std::move(flat);
}

std::vector<TerrainPathPose> TerrainLayer::pathPoses(
  std::vector<std::vector<TerrainPathPose>> * crossings)
{
  nav_msgs::msg::Path plan;
  std::vector<std::vector<TerrainPathPose>> flat;
  std::string flat_frame;
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    plan = plan_;
    if (crossings) {
      flat = crossings_;
      flat_frame = crossings_frame_;
    }
  }
  if (crossings) {
    crossings->clear();
    if (!flat.empty()) {
      // The planner's frame (map), as the plan's.
      try {
        const auto tf = tf_->lookupTransform(global_frame_, flat_frame, tf2::TimePointZero);
        for (auto & c : flat) {
          for (auto & p : c) {
            geometry_msgs::msg::PointStamped in, out;
            in.point.x = p.x;
            in.point.y = p.y;
            in.point.z = p.z;
            tf2::doTransform(in, out, tf);
            p = {out.point.x, out.point.y, out.point.z};
          }
        }
        *crossings = std::move(flat);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "TerrainLayer: cannot transform the plan's crossings (%s) into %s: %s; none waived",
          flat_frame.c_str(), global_frame_.c_str(), ex.what());
      }
    }
  }
  std::vector<TerrainPathPose> out;
  // A level path (a 2D planner's, and the empty one a cancel leaves) says
  // nothing about where the floor goes: no seeds.
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
  out.reserve(plan.poses.size());
  for (const auto & ps : plan.poses) {
    geometry_msgs::msg::PoseStamped in = ps, pose;
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
  // Before the costmap's own mutex: pathPoses() takes TF and the plan's
  // mutex, neither of which needs the costmap.
  std::vector<TerrainPathPose> path;
  std::vector<std::vector<TerrainPathPose>> crossings;
  if (enabled_) {
    path = pathPoses(&crossings);
  }

  std::lock_guard<Costmap::mutex_t> guard(*getMutex());
  std::lock_guard<std::mutex> lock(terrain_mutex_);
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    // After the costmap's own whole-cell rounding, so the two stay aligned.
    terrain_.updateOrigin(getOriginX(), getOriginY());
  }
  if (!enabled_) {
    // Off counts as current, or the costmap never is.
    current_ = true;
    was_reset_ = false;
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  terrain_.setPath(path);
  terrain_.setCrossings(crossings);
  terrain_.classify(
    clock_->now().seconds(), robot_x, robot_y, robot_ground_z_, costmap_, &last_stats_);

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
      "%zu slope, %zu island), %zu step / slope cells waived on the planner's crossings, %llu "
      "samples ray-cleared in all%s",
      last_stats_.known,
      last_stats_.lethal_obstacle + last_stats_.lethal_step + last_stats_.lethal_slope +
        last_stats_.lethal_island,
      last_stats_.lethal_obstacle, last_stats_.lethal_step, last_stats_.lethal_slope,
      last_stats_.lethal_island, last_stats_.waived,
      static_cast<unsigned long long>(terrain_.clearedSamples()),
      has_cloud_ ? "" : " (no cloud yet)");
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

void TerrainLayer::updateCosts(Costmap & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (!current_ && was_reset_) {
    was_reset_ = false;
    current_ = true;
  }
  if (footprint_clearing_enabled_) {
    // The floor under the robot is never seen (and its legs are in the
    // cloud): the footprint is free.
    setConvexPolygonCost(transformed_footprint_, FREE_SPACE);
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace syncai_costmap

PLUGINLIB_EXPORT_CLASS(syncai_costmap::TerrainLayer, syncai_costmap::Layer)
