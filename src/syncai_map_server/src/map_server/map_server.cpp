#include "syncai_map_server/map_server.hpp"

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "syncai_map_server/map_io.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace syncai_map_server
{
MapServer::MapServer(const rclcpp::NodeOptions & options) : Node("map_server", options)
{
  init_parameters();
  init_services();

  // If a map was loaded from the yaml file on startup, publish it once so that
  // late-joining subscribers (transient_local QoS) receive it immediately.
  if (map_available_) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
    occ_pub_->publish(std::move(occ_grid));
  }
}

MapServer::~MapServer()
{
}

void MapServer::init_parameters()
{
  RCLCPP_INFO(this->get_logger(), "[MapServer][%s] Initializing parameters...", __func__);

  this->declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  std::string yaml_filename = this->get_parameter("yaml_filename").as_string();
  RCLCPP_INFO(
    this->get_logger(), "[MapServer][%s] yaml_filename: %s", __func__, yaml_filename.c_str());

  this->declare_parameter("topic_name", "map");
  topic_name_ = this->get_parameter("topic_name").as_string();
  RCLCPP_INFO(this->get_logger(), "[MapServer][%s] topic_name: %s", __func__, topic_name_.c_str());

  this->declare_parameter("frame_id", "map");
  frame_id_ = this->get_parameter("frame_id").as_string();
  RCLCPP_INFO(this->get_logger(), "[MapServer][%s] frame_id: %s", __func__, frame_id_.c_str());

  if (!yaml_filename.empty()) {
    std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp =
      std::make_shared<nav2_msgs::srv::LoadMap::Response>();

    if (!loadMapResponseFromYaml(yaml_filename, rsp)) {
      throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
    }
  } else {
    RCLCPP_INFO(
      get_logger(), "yaml-filename parameter is empty, set map through '%s'-service",
      load_map_service_name_.c_str());
  }
}

void MapServer::init_services()
{
  // Make name prefix for services
  const std::string service_prefix = this->get_name() + std::string("/");

  // Create a service that provides the occupancy grid map
  occ_service_ = this->create_service<nav_msgs::srv::GetMap>(
    service_prefix + std::string(service_name_),
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

  // Create a publisher using the QoS settings to emulate a ROS1 latched topic
  occ_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Create a service that loads the occupancy grid from a file
  load_map_service_ = this->create_service<nav2_msgs::srv::LoadMap>(
    service_prefix + std::string(load_map_service_name_),
    std::bind(&MapServer::loadMapCallback, this, _1, _2, _3));
}

void MapServer::getMapCallback(
  const std::shared_ptr<rmw_request_id_t>, /* request_header */
  const std::shared_ptr<nav_msgs::srv::GetMap::Request> /* request */,
  std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "[MapServer][%s] Handling GetMap request", __func__);
  response->map = msg_;
}

void MapServer::loadMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "[MapServer][%s] Handling LoadMap request", __func__);

  if (!loadMapResponseFromYaml(request->map_url, response)) {
    return;
  }

  auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
  occ_pub_->publish(std::move(occ_grid));
}

bool MapServer::loadMapResponseFromYaml(
  const std::string & yaml_file, std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
{
  switch (loadMapFromYaml(yaml_file, msg_)) {
    case MAP_DOES_NOT_EXIST:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
      return false;
    case INVALID_MAP_METADATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
      return false;
    case INVALID_MAP_DATA:
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
      return false;
    case LOAD_MAP_SUCCESS:
      // Correcting msg_ header when it belongs to specific node
      updateMsgHeader();

      map_available_ = true;
      response->map = msg_;
      response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }

  return true;
}

void MapServer::updateMsgHeader()
{
  msg_.info.map_load_time = now();
  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = now();
}

}  // namespace syncai_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(syncai_map_server::MapServer)
