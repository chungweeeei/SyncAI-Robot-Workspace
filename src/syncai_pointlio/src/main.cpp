#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_pointlio/pointlio_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Single-threaded rclcpp::spin() is load-bearing, not a default left alone:
  // PointLIONode::resetCB takes NO lock against timerCB and relies on every
  // callback sharing the default MutuallyExclusive group, so the timer cannot
  // be halfway through MapBuilder::process() while the builder is swapped out.
  // Moving this node to a MultiThreadedExecutor, or giving the reset service
  // its own callback group, needs a mutex shared with timerCB around m_builder
  // / m_kf first. (pgo_node in SyncAI-Fast-LIO2 does run multi-threaded, for a
  // reason that does not apply here: it has to wait on a client future from
  // inside its handler.)
  rclcpp::spin(std::make_shared<syncai_pointlio::PointLIONode>());
  rclcpp::shutdown();
  return 0;
}
