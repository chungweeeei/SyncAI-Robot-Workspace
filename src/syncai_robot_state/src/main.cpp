#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_robot_state/syncai_robot_state.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Hold the node in a named variable: the executor only keeps a weak_ptr to it,
  // so a temporary would be destroyed before spin() and the node would exit immediately.
  auto node = std::make_shared<syncai_robot_state::RobotStateNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
