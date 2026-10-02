#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_driver_manager/syncai_driver_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<syncai_driver_manager::DriverManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
