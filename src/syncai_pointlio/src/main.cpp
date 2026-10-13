#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "syncai_pointlio/pointlio_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Two threads, one per callback group (2026-10; a single-threaded spin()
  // until then): the sensor subscriptions in their own group, timerCB and
  // resetCB together in the default one. That keeps IMU ingest running while
  // timerCB processes a scan -- under spin() it could not, and the IMU reader
  // dropped samples whenever timerCB outlasted its queue -- while timerCB and
  // resetCB still exclude each other, which resetCB's lock-free builder swap
  // depends on. More threads would buy nothing: both groups are
  // MutuallyExclusive, so a third thread has nothing it is allowed to run.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<syncai_pointlio::PointLIONode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
