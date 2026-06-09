
#include "syncai_util/node_thread.hpp"

#include <memory>

namespace syncai_util
{
NodeThread::NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base)
: node_(node_base)
{
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  thread_ = std::make_unique<std::thread>([&]() {
    executor_->add_node(node_);
    executor_->spin();
    executor_->remove_node(node_);
  });
}

NodeThread::NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor)
: executor_(executor)
{
  thread_ = std::make_unique<std::thread>([&]() { executor_->spin(); });
}

NodeThread::~NodeThread()
{
  executor_->cancel();
  thread_->join();
}
}  // namespace syncai_util
