#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_map_server/costmap_filter_info_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto costmap_filter_info_server_node =
    std::make_shared<syncai_map_server::CostmapFilterInfoServer>();
  rclcpp::spin(costmap_filter_info_server_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
