#ifndef SYNCAI_GLOBAL_PLANNER__PLANNER_SERVER_HPP_
#define SYNCAI_GLOBAL_PLANNER__PLANNER_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_costmap_2d/footprint_collision_checker.hpp"
#include "syncai_nav_core/global_planner.hpp"
#include "syncai_util/node_thread.hpp"
#include "syncai_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_global_planner
{

/**
 * @class syncai_global_planner::PlannerServer
 * @brief An action server implementing the ComputePathToPose interface that
 * hosts global planner plugins to compute plans.
 *
 * Unlike nav2_planner, this node derives from rclcpp::Node. Initialization is
 * split into a two-phase pattern: the constructor only declares parameters and
 * creates the (not yet initialized) global costmap node, and configure() —
 * which must be called after the node is owned by a shared_ptr — performs the
 * rest of the setup (nav2's on_configure + on_activate merged).
 */
class PlannerServer : public rclcpp::Node
{
public:
  using ActionToPose = nav2_msgs::action::ComputePathToPose;
  using ActionServerToPose = syncai_util::SimpleActionServer<ActionToPose>;
  using ActionThroughPoses = nav2_msgs::action::ComputePathThroughPoses;
  using ActionServerThroughPoses = syncai_util::SimpleActionServer<ActionThroughPoses>;
  using PlannerMap = std::unordered_map<std::string, syncai_nav_core::GlobalPlanner::Ptr>;

  /**
   * @brief A constructor for syncai_global_planner::PlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for syncai_global_planner::PlannerServer
   */
  ~PlannerServer();

  /**
   * @brief Second-phase init: initializes the costmap, loads planner plugins
   * and creates the action server. Must be called after the node is owned by
   * a shared_ptr (uses shared_from_this()).
   */
  void configure();

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @param planner_id The planner to use
   * @return Path
   */
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);

protected:
  /**
   * @brief Check if the action server is valid / active
   * @return true if the server is inactive (caller should bail out)
   */
  bool isServerInactive();

  /**
   * @brief Check if the action server has a cancellation request pending;
   * terminates all goals if so.
   * @return true if a cancel was requested
   */
  bool isCancelRequested();

  /// How waitForCostmap() ended.
  enum class CostmapWait {
    Ready,        ///< current, or the (possibly preempted) goal's planner does not read it
    Cancelled,    ///< the goal was cancelled during the wait
    NoStaticMap,  ///< a static layer has never received a map (a 3D-only map)
    TimedOut,     ///< not current within costmap_wait_timeout
  };

  /**
   * @brief Wait for the costmap to be current (updated sensor data, or
   * repopulated after a clear) before a planner that reads it runs.
   *
   * Upstream blocks here with no way out, and on a map without a gridmap the
   * static layer never gets one, so a GridBased goal hung the action thread
   * for good (2026-10-19). The wait now returns on a cancel, takes a preempt
   * (re-checking whether the new goal's planner needs the costmap at all),
   * fails at once while an enabled static layer has never received a map, and
   * gives up after costmap_wait_timeout. Generic over the two action servers.
   * The caller terminates the goal on anything but Ready.
   * @param server The action server whose goal this is
   * @param goal The goal, replaced by an accepted preempt
   * @param reason Set to why the wait failed, for the caller's log line
   */
  template <typename ServerT, typename GoalT>
  CostmapWait waitForCostmap(
    ServerT & server, std::shared_ptr<const GoalT> & goal, std::string & reason);

  /**
   * @brief Name of the first enabled static layer of the global costmap that
   * has never received a map, or "" when there is none.
   */
  std::string staticLayerWithoutMap() const;

  /**
   * @brief Whether the planner getPlan() would pick for this id reads the
   * costmap, i.e. whether a goal for it has to waitForCostmap() first.
   * False for an id getPlan() rejects, which then fails at once.
   */
  bool plannerUsesCostmap(const std::string & planner_id) const;

  /**
   * @brief Check if the action server has a preemption request and replaces
   * the goal with the new preemption goal.
   * @param goal Goal to overwrite (by reference, so the caller sees the
   * preempting goal)
   */
  void getPreemptedGoalIfRequested(std::shared_ptr<const ActionToPose::Goal> & goal);

  /**
   * @brief Get the starting pose from costmap or message, if valid
   * @param goal Goal to find start from
   * @param start The starting pose to use
   * @return bool If successful in finding a valid starting pose
   */
  bool getStartPose(
    std::shared_ptr<const ActionToPose::Goal> goal, geometry_msgs::msg::PoseStamped & start);

  /**
   * @brief Transform start and goal poses into the costmap global frame for
   * path planning plugins to utilize
   * @param curr_start The starting pose to transform
   * @param curr_goal Goal pose to transform
   * @return bool If successful in transforming poses
   */
  bool transformPosesToGlobalFrame(
    geometry_msgs::msg::PoseStamped & curr_start, geometry_msgs::msg::PoseStamped & curr_goal);

  /**
   * @brief Validate that the path contains a meaningful path
   * @param goal Current goal
   * @param path Current path
   * @param planner_id The planner ID used to generate the path
   * @return bool If path is valid
   */
  bool validatePath(
    const geometry_msgs::msg::PoseStamped & goal, const nav_msgs::msg::Path & path,
    const std::string & planner_id);

  /**
   * @brief The action server callback which calls planner to get the path
   */
  void computePlan();

  /**
   * @brief The action server callback which calls planner to get a path through
   * an ordered set of goals (start -> goal[0] -> goal[1] -> ...), concatenating
   * each segment into a single path.
   */
  void computePlanThroughPoses();

  /**
   * @brief The is_path_valid service callback: is the part of a path still
   * ahead of the robot free in the current global costmap? "Free" is two
   * tests per pose: the centre cell below INSCRIBED (Smac's own expansion
   * test) and, with is_path_valid.footprint_check, no LETHAL cell under the
   * padded footprint's perimeter at the path heading +- heading_margin (the
   * test RPP applies on the local costmap before it drives).
   * @param request The path to check (in the costmap's global frame)
   * @param response is_valid, plus the indices of every blocked pose
   */
  void isPathValid(
    const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

  /**
   * @brief Is a LETHAL cell under the perimeter of `footprint` placed at
   * (x, y, theta)? Edge by edge with the collision checker's lineCost(), not
   * footprintCost(): that one ranks NO_INFORMATION (255) above LETHAL (254)
   * and reads an off-map vertex as LETHAL, neither of which is a block here.
   * false also when a vertex is off the map. The costmap mutex must be held.
   */
  bool footprintTouchesLethal(
    double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint);

  /**
   * @brief Publish a path for visualization purposes, and remember it with
   * the id of the planner that made it (is_path_valid asks that planner)
   * @param path Reference to Global Path
   * @param planner_id The goal's planner_id
   */
  void publishPlan(const nav_msgs::msg::Path & path, const std::string & planner_id);

  /**
   * @brief Timer callback: re-send the last plan on `plan`, restamped and
   * trimmed to start at the pose nearest the robot
   */
  void republishPlan();

  /**
   * @brief Forget the last plan and send an empty one on `plan` (repeated by
   * the timer): clear_plan's work, and the idle expiry's in republishPlan()
   */
  void dropPlan();

  /**
   * @brief Service callback: forget the last plan and send an empty one on
   * `plan`, so a viewer stops showing a route nobody is driving any more
   */
  void clearPlan(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters The changed parameters
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerToPose> action_server_pose_;

  // Our action server implements the ComputePathThroughPoses action
  std::unique_ptr<ActionServerThroughPoses> action_server_poses_;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<syncai_nav_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_{0.0};
  // Seconds waitForCostmap() waits for the costmap to be current; <= 0 waits
  // forever (upstream). Guarded by dynamic_params_lock_, like the above.
  double costmap_wait_timeout_{5.0};
  std::string planner_ids_concat_;

  // TF buffer (borrowed from the costmap node)
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap, spun on its own executor thread
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<syncai_util::NodeThread> costmap_thread_;
  syncai_costmap_2d::Costmap2D * costmap_{nullptr};

  // Publisher for the path
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // The last plan handed out, re-sent on `plan` at plan_republish_rate. Written
  // from the action servers' threads, read from the main executor's timer.
  // With it the planner_id it was made with: the is_path_valid request
  // carries no planner id, and the path the BT checks is this plan, so the
  // planner that made it is the one to ask when it does not use the costmap
  // (2026-10-21). Empty until the first plan, and after clear_plan.
  nav_msgs::msg::Path last_plan_;
  std::string last_plan_planner_id_;
  std::mutex last_plan_mutex_;
  rclcpp::TimerBase::SharedPtr plan_republish_timer_;

  // Empty plans the timer still owes `plan` after a clear_plan, under
  // last_plan_mutex_. The backend reads `plan` BEST_EFFORT, so one empty
  // message can be dropped and leave the console showing the finished route
  // for good; a few repeats make that unlikely without sending an empty path
  // every second for as long as the robot is idle. A new plan zeroes it.
  int clear_resends_left_{0};

  // When the last plan was made or last checked by is_path_valid, under
  // last_plan_mutex_; republishPlan() drops a plan unused for
  // plan_idle_clear_time (s, 0 = never), the end of a failed navigation's
  // route.
  std::chrono::steady_clock::time_point last_plan_used_;
  double plan_idle_clear_time_{30.0};

  // Called by the task_runner when a navigation succeeds or is cancelled, and
  // by Octo3D on a map switch
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_plan_service_;

  // Lets the BT keep its current path until the costmap blocks it, instead of
  // replacing it with whatever a fresh plan returns every RateController tick
  rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr is_path_valid_service_;

  // Perimeter walk for is_path_valid over the global costmap (set up in
  // configure(), once costmap_ exists)
  syncai_costmap_2d::FootprintCollisionChecker<syncai_costmap_2d::Costmap2D *> collision_checker_;

  // is_path_valid.footprint_check / heading_margin. Read on the main executor
  // in isPathValid(), written by the parameter callback under
  // dynamic_params_lock_, which isPathValid() deliberately does not take (a
  // plan in progress holds that lock for its whole search)
  std::atomic<bool> path_check_footprint_{true};
  std::atomic<double> path_check_heading_margin_{0.35};

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;
};

}  // namespace syncai_global_planner

#endif  // SYNCAI_GLOBAL_PLANNER__PLANNER_SERVER_HPP_
