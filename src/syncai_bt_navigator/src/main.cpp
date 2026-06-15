#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_bt_navigator/bt_navigator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_bt_navigator::BtNavigator>();

  // configure() relies on shared_from_this(), so it can only run after the node
  // has been wrapped in a shared_ptr above (not from the constructor).
  if (!node->configure()) {
    RCLCPP_FATAL(node->get_logger(), "Failed to configure bt_navigator, shutting down");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);

  node->cleanup();
  rclcpp::shutdown();
  return 0;
}
