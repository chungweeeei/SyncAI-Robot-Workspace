#ifndef NAV2_UTIL__NODE_THREAD_HPP_
#define NAV2_UTIL__NODE_THREAD_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace syncai_util
{

/**
 * @brief A background thread to process node callbacks
 */
class NodeThread
{
public:
  /**
   * @brief A background thread to process node callbacks constructor
   * @param node_base Interface to Node to spin in thread
   */
  explicit NodeThread(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base);

  /**
   * @brief A background thread to process executor's callbacks constructor
   * @param executor Interface to executor to spin in thread
   */
  explicit NodeThread(rclcpp::executors::SingleThreadedExecutor::SharedPtr executor);

  /**
   * @brief A background thread to process node callbacks constructor
   * @param node Node pointer to spin in thread
   */
  template <typename NodeT>
  explicit NodeThread(NodeT node) : NodeThread(node->get_node_base_interface())
  {
  }

  /**
   * @brief A destructor
   */
  ~NodeThread();

protected:
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  std::unique_ptr<std::thread> thread_;
  rclcpp::Executor::SharedPtr executor_;
};
}  // namespace syncai_util

#endif  // NAV2_UTIL__NODE_THREAD_HPP_