#include "syncai_map_server/map_saver.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

using namespace std::placeholders;

namespace syncai_map_server
{
MapSaver::MapSaver(const rclcpp::NodeOptions & options) : Node("map_saver_server", options)
{
  RCLCPP_INFO(this->get_logger(), "[MapSaver][%s] Initializing MapSaver node...", __func__);

  init_parameters();
  init_services();
}

MapSaver::~MapSaver()
{
}

void MapSaver::init_parameters()
{
  this->declare_parameter("save_map_timeout", 5.0);
  save_map_timeout_ = std::make_shared<rclcpp::Duration>(
    rclcpp::Duration::from_seconds(this->get_parameter("save_map_timeout").as_double()));

  this->declare_parameter("free_thresh_default", 0.25);
  free_thresh_default_ = this->get_parameter("free_thresh_default").as_double();

  this->declare_parameter("occupied_thresh_default", 0.65);
  occupied_thresh_default_ = this->get_parameter("occupied_thresh_default").as_double();

  this->declare_parameter("map_subscribe_transient_local", true);
  map_subscribe_transient_local_ = this->get_parameter("map_subscribe_transient_local").as_bool();
}

void MapSaver::init_services()
{
  const std::string service_prefix = get_name() + std::string("/");
  save_map_service_ = this->create_service<nav2_msgs::srv::SaveMap>(
    service_prefix + std::string(save_map_service_name_),
    std::bind(&MapSaver::saveMapCallback, this, _1, _2, _3));
}

void MapSaver::saveMapCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<nav2_msgs::srv::SaveMap::Request> request,
  std::shared_ptr<nav2_msgs::srv::SaveMap::Response> response)
{
  // Set input arguments and call saveMapTopicToFile()
  SaveParameters save_parameters;
  save_parameters.map_file_name = request->map_url;
  save_parameters.image_format = request->image_format;
  save_parameters.free_thresh = request->free_thresh;
  save_parameters.occupied_thresh = request->occupied_thresh;

  try {
    save_parameters.mode = map_mode_from_string(request->map_mode);
  } catch (std::invalid_argument &) {
    save_parameters.mode = MapMode::Trinary;
    RCLCPP_WARN(
      this->get_logger(), "Map mode parameter not recognized: '%s', using default value (trinary)",
      request->map_mode.c_str());
  }

  response->result = saveMapTopicToFile(request->map_topic, save_parameters);
}

bool MapSaver::saveMapTopicToFile(
  const std::string & map_topic, const SaveParameters & save_parameters)
{
  // Local copies of map_topic and save_parameters that could be changed
  std::string map_topic_loc = map_topic;
  SaveParameters save_parameters_loc = save_parameters;

  RCLCPP_INFO(
    this->get_logger(), "Saving map from \'%s\' topic to \'%s\' file", map_topic_loc.c_str(),
    save_parameters_loc.map_file_name.c_str());

  try {
    // Correct map_topic_loc if necessary
    if (map_topic_loc == "") {
      map_topic_loc = "map";
      RCLCPP_WARN(
        this->get_logger(), "Map topic unspecified. Map messages will be read from \'%s\' topic",
        map_topic_loc.c_str());
    }

    // Set default for MapSaver node thresholds parameters
    if (save_parameters_loc.free_thresh == 0.0) {
      RCLCPP_WARN(
        this->get_logger(), "Free threshold unspecified. Setting it to default value: %f",
        free_thresh_default_);
      save_parameters_loc.free_thresh = free_thresh_default_;
    }
    if (save_parameters_loc.occupied_thresh == 0.0) {
      RCLCPP_WARN(
        this->get_logger(), "Occupied threshold unspecified. Setting it to default value: %f",
        occupied_thresh_default_);
      save_parameters_loc.occupied_thresh = occupied_thresh_default_;
    }

    std::promise<nav_msgs::msg::OccupancyGrid::SharedPtr> prom;
    std::future<nav_msgs::msg::OccupancyGrid::SharedPtr> future_result = prom.get_future();
    // A callback function that receives map message from subscribed topic
    auto mapCallback = [&prom](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
      prom.set_value(msg);
    };

    rclcpp::QoS map_qos(10);  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1);
    }

    // Create new CallbackGroup for map_sub
    auto callback_group =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    auto option = rclcpp::SubscriptionOptions();
    option.callback_group = callback_group;
    auto map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_loc, map_qos, mapCallback, option);

    // Create SingleThreadedExecutor to spin map_sub in callback_group
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(callback_group, this->get_node_base_interface());
    // Spin until map message received
    auto timeout = save_map_timeout_->to_chrono<std::chrono::nanoseconds>();
    auto status = executor.spin_until_future_complete(future_result, timeout);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to spin map subscription");
      return false;
    }
    // map_sub is no more needed
    map_sub.reset();
    // Map message received. Saving it to file
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg = future_result.get();
    if (saveMapToFile(*map_msg, save_parameters_loc)) {
      RCLCPP_INFO(this->get_logger(), "Map saved successfully");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save the map");
      return false;
    }
  } catch (std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save the map: %s", e.what());
    return false;
  }

  return false;
}

}  // namespace syncai_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(syncai_map_server::MapSaver)