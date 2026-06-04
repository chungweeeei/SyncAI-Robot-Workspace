#ifndef SYNCAI_MAP_SERVER__MAP_SAVER_HPP_
#define SYNCAI_MAP_SERVER__MAP_SAVER_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/srv/save_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_map_server/map_io.hpp"

namespace syncai_map_server
{
class MapSaver : public rclcpp::Node
{
public:
  explicit MapSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapSaver();

  bool saveMapTopicToFile(const std::string & map_topic, const SaveParameters & save_parameters);

private:
  void init_parameters();
  void init_services();

  void saveMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response);

  // The timeout for saving the map in service
  std::shared_ptr<rclcpp::Duration> save_map_timeout_;

  // Default values for map thresholds
  double free_thresh_default_;
  double occupied_thresh_default_;

  // param for handling QoS configuration
  bool map_subscribe_transient_local_;

  // The name of the service for saving a map from topic
  const std::string save_map_service_name_{"save_map"};
  // A service to save the map to a file at run time (SaveMap)
  rclcpp::Service<nav2_msgs::srv::SaveMap>::SharedPtr save_map_service_;
};
}  // namespace syncai_map_server

#endif  // SYNCAI_MAP_SERVER__MAP_SAVER_HPP_