#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_planner/planner_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_planner::PlannerServer>();
  // Second-phase init: safe to use shared_from_this() now that the node is
  // owned by a shared_ptr.
  node->configure();

  // A SingleThreadedExecutor suffices here: the ComputePathToPose execute
  // callback (including the blocking waitForCostmap) runs on the
  // SimpleActionServer's own spin thread, and the costmap node is spun by its
  // own NodeThread. This executor only services parameter callbacks.
  // Note: do NOT add the costmap node here — its NodeThread already spins it.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
