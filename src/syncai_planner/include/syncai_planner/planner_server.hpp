#ifndef SYNCAI_PLANNER__PLANNER_SERVER_HPP_
#define SYNCAI_PLANNER__PLANNER_SERVER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"
#include "syncai_nav_core/global_planner.hpp"
#include "syncai_util/node_thread.hpp"
#include "syncai_util/simple_action_server.hpp"
#include "tf2_ros/buffer.h"

namespace syncai_planner
{

/**
 * @class syncai_planner::PlannerServer
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
   * @brief A constructor for syncai_planner::PlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief A destructor for syncai_planner::PlannerServer
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

  /**
   * @brief Wait for costmap to be valid with updated sensor data, or to
   * repopulate after a clearing recovery. Blocks until true without timeout.
   */
  void waitForCostmap();

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
   * @brief Publish a path for visualization purposes
   * @param path Reference to Global Path
   */
  void publishPlan(const nav_msgs::msg::Path & path);

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
  std::string planner_ids_concat_;

  // TF buffer (borrowed from the costmap node)
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap, spun on its own executor thread
  std::shared_ptr<syncai_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<syncai_util::NodeThread> costmap_thread_;
  syncai_costmap_2d::Costmap2D * costmap_{nullptr};

  // Publisher for the path
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;
};

}  // namespace syncai_planner

#endif  // SYNCAI_PLANNER__PLANNER_SERVER_HPP_
