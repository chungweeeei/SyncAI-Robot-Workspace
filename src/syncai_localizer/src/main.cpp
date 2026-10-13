#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_localizer/localizer_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Named, NOT `add_node(std::make_shared<LocalizerNode>())`: Executor::add_node
  // stores a weak_ptr, so a temporary shared_ptr dies at the end of that
  // statement and takes the node with it -- the process then spins over an
  // empty node set, broadcasting no TF, while looking alive in `ps`. (Same
  // note as syncai_mapping's pgo_main.cpp, where that bug actually happened.)
  auto node = std::make_shared<syncai_localizer::LocalizerNode>();
  // Three threads: one for the timer/subscriber group, one for the services'
  // group (relocalize, relocalize_check, auto_localize, reset_odometry,
  // initialpose), so the TF rebroadcast keeps running while relocalize's
  // loadMap -- several seconds on a large map -- is in progress; and one for
  // reset_odometry's ResetLIO client group, whose response must be delivered
  // while the service handler blocks on it (with two, it would queue behind
  // the timer's GICP rounds on the only free thread). ICPLocalizer::m_target_mutex
  // exists for the first split: loadMap swaps the targets under it, align
  // holds it for one round. Same three-group shape as pgo_node, whose
  // ResetLIO client is the model for this one.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
