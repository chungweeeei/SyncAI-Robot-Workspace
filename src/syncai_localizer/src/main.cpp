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
  // Two threads: one for the timer/subscriber group, one for the services'
  // group (relocalize, relocalize_check, initialpose), so the TF rebroadcast
  // keeps running while relocalize's loadMap -- several seconds on a large
  // map -- is in progress. ICPLocalizer::m_target_mutex exists for exactly
  // this split: loadMap swaps the targets under it, align holds it for one
  // round. This is the opposite decision from syncai_pointlio's
  // single-threaded spin, and the model pgo_node's three-thread arrangement
  // was copied from.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
