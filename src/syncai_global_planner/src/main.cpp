#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_global_planner/planner_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_global_planner::PlannerServer>();
  // Second-phase init: safe to use shared_from_this() now that the node is
  // owned by a shared_ptr.
  node->configure();

  // configure() returns early on a shutdown that arrives while the costmap is
  // still waiting for its first transform (CostmapROS::activate()). Building
  // the executor on the dead context then throws RCLError "failed to create
  // guard condition" out of main(), and the process aborts (exit 134) instead
  // of exiting cleanly.
  if (!rclcpp::ok()) {
    return 0;
  }

  // A SingleThreadedExecutor suffices here: each planner action's execute
  // callback runs on its SimpleActionServer's own spin thread, Octo3D's
  // load_map / load_keepout on theirs, and the costmap node is spun by its
  // own NodeThread. This executor serves what is left, all of it short:
  // is_path_valid, clear_plan, Octo3D's clear_overlay, the plan republish
  // timer and the parameter services -- one thread, which is what lets those
  // callbacks share state without a lock (Octo3D's isPathValid() reads members
  // its parameter callback writes).
  // Note: do NOT add the costmap node here — its NodeThread already spins it.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
