#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_mapping/pgo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Named, NOT `add_node(std::make_shared<PGONode>())`. Executor::add_node
  // stores a weak_ptr, so a temporary shared_ptr dies at the end of that
  // statement and takes the node with it -- the process then spins forever over
  // an empty node set, publishing nothing and broadcasting no TF, while looking
  // alive in `ps`. (rclcpp::spin(make_shared<...>()), which this replaced, is
  // safe only because the argument outlives the call.) The symptom is a console
  // with no point cloud and a DDS "Finis." a third of a second after startup.
  auto node = std::make_shared<syncai_mapping::PGONode>();
  // Three threads for three groups, the same arrangement (and the same reason)
  // as syncai_localizer's localizer_node: the default group keeps the timer, both
  // subscriptions and save_maps serialised exactly as rclcpp::spin() did, so
  // nothing about the steady-state behaviour moves. The other two exist purely
  // so resetMappingCB can block on a client future without deadlocking itself
  // -- the handler runs on one, its response arrives on the other. This is
  // the opposite decision from syncai_pointlio's single-threaded spin, and
  // m_pgo_mutex exists because of it; see its declaration for the discipline.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
