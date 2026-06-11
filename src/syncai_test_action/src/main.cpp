#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_test_action/test_action_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_test_action::TestActionServer>();
  // Second-phase init: safe to use shared_from_this() now that the node is
  // owned by a shared_ptr.
  node->initialize();

  // The execute callback runs on the SimpleActionServer's own spin thread,
  // so a SingleThreadedExecutor is enough for the rest of the node.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
