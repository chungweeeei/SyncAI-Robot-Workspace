#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_ros.hpp"

// Standalone runner for the Costmap2DROS wrapper.
//
// Costmap2DROS is a plain rclcpp::Node (not a LifecycleNode), so the
// configure/activate steps are driven manually here:
//   1. construct (declares parameters)
//   2. init()      -> manual on_configure (tf, plugins, publishers, executor)
//   3. activate()  -> manual on_activate  (map update thread, dynamic params)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<syncai_costmap_2d::Costmap2DROS>("costmap");
  node->init();
  node->activate();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
