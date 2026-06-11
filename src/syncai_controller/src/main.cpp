#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_controller/controller_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_controller::ControllerServer>();
  // Second-phase init: safe to use shared_from_this() now that the node is
  // owned by a shared_ptr.
  node->configure();

  // A SingleThreadedExecutor suffices here: the FollowPath execute callback
  // runs on the SimpleActionServer's own spin thread, and the costmap node is
  // spun by its own NodeThread. This executor services the odom and
  // speed-limit subscriptions plus parameter callbacks.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
