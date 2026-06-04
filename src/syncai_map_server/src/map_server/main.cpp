#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_map_server/map_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto map_server_node = std::make_shared<syncai_map_server::MapServer>();
  rclcpp::spin(map_server_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
