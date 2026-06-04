#ifndef SYNCAI_MAP_SERVER__MAP_SERVER_HPP_
#define SYNCAI_MAP_SERVER__MAP_SERVER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "nav2_msgs/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

namespace syncai_map_server
{
class MapServer : public rclcpp::Node
{
public:
  explicit MapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapServer();

private:
  void init_parameters();
  void init_services();

  bool loadMapResponseFromYaml(
    const std::string & yaml_file, std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  void updateMsgHeader();

  void getMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
    std::shared_ptr<nav_msgs::srv::GetMap::Response> response);

  void loadMapCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response);

  // The name of the service for getting a map
  const std::string service_name_{"map"};

  // The name of the service for loading a map
  const std::string load_map_service_name_{"load_map"};

  // A service to provide the occupancy grid (GetMap) and the message to return
  rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr occ_service_;

  // A service to load the occupancy grid from file at run time (LoadMap)
  rclcpp::Service<nav2_msgs::srv::LoadMap>::SharedPtr load_map_service_;

  // A topic on which the occupancy grid will be published
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  // The name of the topic on which the occupancy grid is published
  std::string topic_name_;

  // The frame ID used in the returned OccupancyGrid message
  std::string frame_id_;

  // The message to publish on the occupancy grid topic
  nav_msgs::msg::OccupancyGrid msg_;

  // true if msg_ was initialized
  bool map_available_{false};
};
}  // namespace syncai_map_server

#endif  // SYNCAI_MAP_SERVER__MAP_SERVER_HPP_