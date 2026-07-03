#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_driver_manager/syncai_driver_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<syncai_driver_manager::DriverManagerNode>());
  rclcpp::shutdown();
  return 0;
}
