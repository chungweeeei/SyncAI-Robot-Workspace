#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_lio_bridge/lio_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Single-threaded is enough: the two subscription callbacks only latch the
  // newest sample and the timer does all the work, so there is nothing to
  // overlap.
  rclcpp::spin(std::make_shared<syncai_lio_bridge::LioBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
