#ifndef SYNCAI_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
#define SYNCAI_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_

#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace syncai_map_server
{

/**
 * @class CostmapFilterInfoServer
 * @brief Publishes the CostmapFilterInfo message consumed by costmap filters
 * (e.g. syncai_costmap_2d::KeepoutFilter). The message describes which topic
 * carries the filter mask and how to convert the mask's OccupancyGrid values
 * into filter-space values (base/multiplier). Published once, latched
 * (transient_local), mirroring nav2_map_server::CostmapFilterInfoServer but
 * as a plain (non-lifecycle) node.
 */
class CostmapFilterInfoServer : public rclcpp::Node
{
public:
  explicit CostmapFilterInfoServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CostmapFilterInfoServer();

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};

}  // namespace syncai_map_server

#endif  // SYNCAI_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
