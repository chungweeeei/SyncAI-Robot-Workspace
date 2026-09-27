#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_mapping/hba_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Single-threaded, unlike pgo_node's three-thread executor in pgo_main.cpp:
  // nothing here blocks on a client, and the whole optimisation runs inside
  // the timer callback on purpose -- an offline job that holds the executor
  // for seconds is fine when the node serves nothing else meanwhile.
  rclcpp::spin(std::make_shared<syncai_mapping::HBANode>());
  rclcpp::shutdown();
  return 0;
}
