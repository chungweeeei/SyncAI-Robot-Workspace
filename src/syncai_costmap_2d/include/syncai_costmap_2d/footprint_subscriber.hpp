#ifndef SYNCAI_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_
#define SYNCAI_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/footprint.hpp"
#include "syncai_util/robot_utils.hpp"

namespace syncai_costmap_2d
{
/**
 * @class FootprintSubscriber
 * @brief Subscriber to the footprint topic to get current robot footprint
 * (if changing) for use in collision avoidance
 */

class FootprintSubscriber
{
public:
  /**
   * @brief A constructor
   */
  FootprintSubscriber(
    const rclcpp::Node::SharedPtr & node, const std::string & topic_name, tf2_ros::Buffer & tf,
    std::string robot_base_frame = "base_link", double transform_timeout = 0.1);

  /**
   * @brief A destructor
   */
  ~FootprintSubscriber() {}

  /**
   * @brief Returns the latest robot footprint, in the form as received from topic (oriented).
   *
   * @param footprint Output param. Latest received footprint
   * @param footprint_header Output param. Header associated with the footprint
   * @return False if no footprint has been received
   */
  bool getFootprintRaw(
    std::vector<geometry_msgs::msg::Point> & footprint, std_msgs::msg::Header & footprint_header);

  /**
   * @brief Returns the latest robot footprint, transformed into robot base frame (unoriented).
   *
   * @param footprint Output param. Latest received footprint, unoriented
   * @param footprint_header Output param. Header associated with the footprint
   * @return False if no footprint has been received or if transformation failed
   */
  bool getFootprintInRobotFrame(
    std::vector<geometry_msgs::msg::Point> & footprint, std_msgs::msg::Header & footprint_header);

protected:
  /**
   * @brief Callback to process new footprint updates.
   */
  void footprint_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);

  tf2_ros::Buffer & tf_;
  std::string robot_base_frame_;
  double transform_tolerance_;
  bool footprint_received_{false};
  geometry_msgs::msg::PolygonStamped::SharedPtr footprint_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
};
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__FOOTPRINT_SUBSCRIBER_HPP_