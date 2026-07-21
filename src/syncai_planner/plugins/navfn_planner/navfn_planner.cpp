#include "syncai_planner/plugins/navfn_planner/navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "syncai_costmap_2d/cost_values.hpp"
#include "syncai_planner/plugins/navfn_planner/navfn.hpp"
#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using syncai_util::declare_parameter_if_not_declared;

namespace syncai_planner
{
NavfnPlanner::NavfnPlanner() : tf_(nullptr), costmap_(nullptr)
{
}

NavfnPlanner::~NavfnPlanner()
{
  RCLCPP_INFO(
    logger_, "[NavfnPlanner][%s]  Destroying plugin %s of type NavfnPlanner", __func__,
    name_.c_str());
}

void NavfnPlanner::initialize(
  const rclcpp::Node::SharedPtr & node, std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  name_ = name;

  // Get the costmap from the costmap_ros object and store the global frame
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Initialize the clock and logger
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  RCLCPP_INFO(
    logger_, "[NavfnPlanner][%s] Configuring plugin %s of type NavfnPlanner", __func__,
    name_.c_str());

  // Initialize parameters, declare this plugin's parameters.
  declare_parameter_if_not_declared(node, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name + ".tolerance", tolerance_);
  RCLCPP_INFO(logger_, "[NavfnPlanner][%s] Tolerance set to %.2f", __func__, tolerance_);

  declare_parameter_if_not_declared(node, name + ".use_astar", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_astar", use_astar_);
  RCLCPP_INFO(
    logger_, "[NavfnPlanner][%s] use_astar set to %s", __func__, use_astar_ ? "true" : "false");

  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  RCLCPP_INFO(
    logger_, "[NavfnPlanner][%s] allow_unknown set to %s", __func__,
    allow_unknown_ ? "true" : "false");

  declare_parameter_if_not_declared(
    node, name + ".use_final_approach_orientation", rclcpp::ParameterValue(false));
  node->get_parameter(name + ".use_final_approach_orientation", use_final_approach_orientation_);
  RCLCPP_INFO(
    logger_, "[NavfnPlanner][%s] use_final_approach_orientation set to %s", __func__,
    use_final_approach_orientation_ ? "true" : "false");

  // Create a planner based on the new costmap size
  // TODO: learn more about Navfn Planner
  planner_ = std::make_unique<NavFn>(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
}

nav_msgs::msg::Path NavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  // Declare a path variable to be returned at the end of the function.
  nav_msgs::msg::Path path;

  // Corner case of the start(x,y) = goal(x,y)
  if (
    start.pose.position.x == goal.pose.position.x &&
    start.pose.position.y == goal.pose.position.y) {
    // check cost
    unsigned int mx, my;

    /**
     * costmap -> worldToMap 是將世界座標(map)的位置轉成格子的index
     */
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);

    // 透過轉出來的index確認在該格子上的cost是否有障礙物
    if (costmap_->getCost(mx, my) == syncai_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(
        logger_, "[NavfnPlanner][%s] Failed to create a unique pose path because of obstacles",
        __func__);

      // 如果是障礙物就直接 return empty path
      return path;
    }

    // when goal pose is equal to start pose, we will return a path with only one pose, which is the start/goal pose.
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;
    // if we have a different start and goal orientation, set the unique path pose to the goal
    // orientation, unless use_final_approach_orientation=true where we need it to be the start
    // orientation to avoid movement from the local planner
    if (start.pose.orientation != goal.pose.orientation && !use_final_approach_orientation_) {
      pose.pose.orientation = goal.pose.orientation;
    }
    path.poses.push_back(pose);
    return path;
  }

  /**
   * start generate global path
   * @param tolerance 是「goal 到不了的時候,允許退而求其次的範圍」- 以公尺為單位，定義在 goal 周圍可以接受的替代落點距離。
   */
  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
    RCLCPP_WARN(
      logger_, "[NavfnPlanner][%s] %s: failed to create plan with tolerance %.2f.", __func__,
      name_.c_str(), tolerance_);
  }

  return path;
}

bool NavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan path, just in case
  plan.poses.clear();

  plan.header.stamp = clock_->now();
  plan.header.frame_id = global_frame_;

  // declare variable to store start position
  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_INFO(
    logger_, "[NavfnPlanner][%s] Making plan from (%.2f,%.2f) to (%.2f,%.2f)", __func__,
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "[NavfnPlanner][%s] Cannot create a plan: the robot's start position is out off the global"
      " costmap. Planning will always fail.",
      __func__);
    return false;
  }

  // clear the starting cell(robot its own pose) within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  // make sure to lock the costmap while we read from it.
  std::unique_lock<syncai_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  // make sure to resize the underlying array that Navfn uses
  planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
  planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);
  lock.unlock();

  int map_start[2];  // store convert map index into an array to feed into Navfn Planner
  map_start[0] = mx;
  map_start[1] = my;

  // update wx/wy variables to the goal position for later use
  wx = goal.position.x;
  wy = goal.position.y;

  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "[NavfnPlanner][%s] The goal sent to the planner is off the global costmap."
      " Planning will always fail to this goal.",
      __func__);
    return false;
  }

  int map_goal[2];  // store convert map index into an array to feed into Navfn Planner
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);

  /**
   * calcNavFnAstar() 或 calcNavFnDijkstra() 並非直接計算出可走的path, 
   * 而是計算potential field, 也就是從 goal 開始往外擴散的 cost gradient。
   */
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;
  bool found_legal = false;

  // 計算完potential field後，檢查goal是否reachable
  p = goal;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
  } else {
    // Goal is not reachable. Trying to find nearest to the goal reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squaredDistance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
  }

  // whether goal pose is reachable or not
  if (!found_legal) {
    RCLCPP_WARN(
      logger_, "[NavfnPlanner][%s] Failed to find a legal pose within tolerance %.2f of the goal.",
      __func__, tolerance);
    return !plan.poses.empty();
  }

  // whether can extract a plan to the best pose found
  // extract the plan
  if (getPlanFromPotential(best_pose, plan)) {
    // 把plan的終點修成「精確的 goal 座標 」，因為剛從grid map extract出來的plan，終點會是「離 goal 最近的可行走點」，但不一定是「精確的 goal 座標 」
    smoothApproachToGoal(best_pose, plan);

    // If use_final_approach_orientation=true, interpolate the last pose orientation from the
    // previous pose to set the orientation to the 'final approach' orientation of the robot so
    // it does not rotate.
    // And deal with corner case of plan of length 1
    if (use_final_approach_orientation_) {
      size_t plan_size = plan.poses.size();
      if (plan_size == 1) {
        plan.poses.back().pose.orientation = start.orientation;
      } else if (plan_size > 1) {
        double dx, dy, theta;
        auto last_pose = plan.poses.back().pose.position;
        auto approach_pose = plan.poses[plan_size - 2].pose.position;
        // Deal with the case of NavFn producing a path with two equal last poses
        if (
          std::abs(last_pose.x - approach_pose.x) < 0.0001 &&
          std::abs(last_pose.y - approach_pose.y) < 0.0001 && plan_size > 2) {
          approach_pose = plan.poses[plan_size - 3].pose.position;
        }
        dx = last_pose.x - approach_pose.x;
        dy = last_pose.y - approach_pose.y;
        theta = atan2(dy, dx);
        plan.poses.back().pose.orientation =
          syncai_util::geometry_utils::orientationAroundZAxis(theta);
      }
    }
  } else {
    RCLCPP_ERROR(
      logger_,
      "[NavfnPlanner][%s] Failed to create a plan from potential when a legal"
      " potential was found. This shouldn't happen.",
      __func__);
  }

  return !plan.poses.empty();
}

void NavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.

  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squaredDistance(last_pose.pose, second_to_last_pose.pose) >
      squaredDistance(goal, second_to_last_pose.pose)) {
      plan.poses.back().pose = goal;
      return;
    }
  }

  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  goal_copy.header = plan.header;
  plan.poses.push_back(goal_copy);
}

bool NavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal, nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      logger_,
      "[NavfnPlanner][%s] The goal sent to the navfn planner is off the global costmap."
      " Planning will always fail to this goal.",
      __func__);
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY())
                             ? (costmap_->getSizeInCellsX() * 4)
                             : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
  RCLCPP_DEBUG(logger_, "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double NavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

bool NavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  // check if in bounds
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    logger_, "[NavfnPlanner][%s] worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", __func__,
    mx, my, costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void NavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void NavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  costmap_->setCost(mx, my, syncai_costmap_2d::FREE_SPACE);
}

}  // namespace syncai_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(syncai_planner::NavfnPlanner, syncai_nav_core::GlobalPlanner)
