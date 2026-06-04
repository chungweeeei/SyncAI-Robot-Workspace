#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "syncai_map_server/map_saver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto service_node = std::make_shared<syncai_map_server::MapSaver>();
  rclcpp::spin(service_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}