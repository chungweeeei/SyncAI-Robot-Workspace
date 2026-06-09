#ifndef SYNCAI_NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__COSTMAP_SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d.hpp"

namespace syncai_costmap_2d
{
/**
 * @class CostmapSubscriber
 * @brief Subscribes to the costmap via a ros topic
 */
class CostmapSubscriber
{
public:
  /**
   * @brief A constructor
   */
  CostmapSubscriber(const rclcpp::Node::SharedPtr & node, const std::string & topic_name);

  /**
    * @brief A destructor
    */
  ~CostmapSubscriber();

  /**
   * @brief A Get the costmap from topic
   */
  std::shared_ptr<Costmap2D> getCostmap();

  /**
   * @brief Convert an occ grid message into a costmap object
   */
  void toCostmap2D();

  /**
   * @brief Callback for the costmap topic
   */
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

protected:
  std::shared_ptr<Costmap2D> costmap_;
  nav_msgs::msg::Costmap::SharedPtr costmap_msg_;
  std::string topic_name_;
  bool costmap_received_{false};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
};
}  // namespace syncai_costmap_2d

#endif