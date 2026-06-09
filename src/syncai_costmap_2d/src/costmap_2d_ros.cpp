#include "syncai_costmap_2d/costmap_2d_ros.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "syncai_costmap_2d/footprint.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/robot_utils.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace syncai_costmap_2d
{
Costmap2DROS::Costmap2DROS(const rclcpp::NodeOptions & options)
: Costmap2DROS("costmap", "/", "costmap")
{
  (void)options;
}

Costmap2DROS::Costmap2DROS(const std::string & name) : Costmap2DROS(name, "/", name)
{
}

Costmap2DROS::Costmap2DROS(
  const std::string & name, const std::string & parent_namespace,
  const std::string & local_namespace)
: rclcpp::Node(name),
  name_(name),
  parent_namespace_(parent_namespace),
  default_plugins_{"static_layer"},
  default_types_{"syncai_costmap_2d::StaticLayer"}
{
  (void)local_namespace;  // TODO(syncai): wire namespace handling in configure() (Step 2)
  RCLCPP_INFO(this->get_logger(), "Creating Costmap");

  this->declare_parameter("always_send_full_costmap", rclcpp::ParameterValue(false));
  this->declare_parameter("footprint_padding", rclcpp::ParameterValue(0.01f));
  this->declare_parameter("footprint", rclcpp::ParameterValue(std::string("[]")));
  this->declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
  this->declare_parameter("height", rclcpp::ParameterValue(5));
  this->declare_parameter("width", rclcpp::ParameterValue(5));
  this->declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));

  this->declare_parameter(
    "map_topic",
    rclcpp::ParameterValue(
      (parent_namespace_ == "/" ? "/" : parent_namespace_ + "/") + std::string("map")));

  this->declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  this->declare_parameter("origin_x", rclcpp::ParameterValue(0.0));
  this->declare_parameter("origin_y", rclcpp::ParameterValue(0.0));
  this->declare_parameter("plugins", rclcpp::ParameterValue(default_plugins_));
  this->declare_parameter("filters", rclcpp::ParameterValue(std::vector<std::string>()));
  this->declare_parameter("publish_frequency", rclcpp::ParameterValue(1.0));
  this->declare_parameter("resolution", rclcpp::ParameterValue(0.1));
  this->declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  this->declare_parameter("robot_radius", rclcpp::ParameterValue(0.1));
  this->declare_parameter("rolling_window", rclcpp::ParameterValue(false));
  this->declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
  this->declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
  this->declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
  this->declare_parameter(
    "unknown_cost_value", rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
  this->declare_parameter("update_frequency", rclcpp::ParameterValue(5.0));
  this->declare_parameter("use_maximum", rclcpp::ParameterValue(false));
}

Costmap2DROS::~Costmap2DROS()
{
}

void Costmap2DROS::getParameters()
{
  RCLCPP_DEBUG(this->get_logger(), "Getting parameters");

  this->get_parameter("always_send_full_costmap", always_send_full_costmap_);
  this->get_parameter("footprint", footprint_);
  this->get_parameter("footprint_padding", footprint_padding_);
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("height", map_height_meters_);
  this->get_parameter("origin_x", origin_x_);
  this->get_parameter("origin_y", origin_y_);
  this->get_parameter("publish_frequency", map_publish_frequency_);
  this->get_parameter("resolution", resolution_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("robot_radius", robot_radius_);
  this->get_parameter("rolling_window", rolling_window_);
  this->get_parameter("track_unknown_space", track_unknown_space_);
  this->get_parameter("transform_tolerance", transform_tolerance_);
  this->get_parameter("update_frequency", map_update_frequency_);
  this->get_parameter("width", map_width_meters_);
  this->get_parameter("plugins", plugin_names_);
  this->get_parameter("filters", filter_names_);

  if (plugin_names_ == default_plugins_) {
    for (size_t i = 0; i < default_plugins_.size(); ++i) {
      syncai_util::declare_parameter_if_not_declared(
        this, default_plugins_[i] + ".plugin", rclcpp::ParameterValue(default_types_[i]));
    }
  }
  plugin_types_.resize(plugin_names_.size());
  filter_types_.resize(filter_names_.size());

  // 1. All plugins must have 'plugin' param defined in their namespace to define the plugin type
  for (size_t i = 0; i < plugin_names_.size(); ++i) {
    plugin_types_[i] = syncai_util::get_plugin_type_param(this, plugin_names_[i]);
  }
  for (size_t i = 0; i < filter_names_.size(); ++i) {
    filter_types_[i] = syncai_util::get_plugin_type_param(this, filter_names_[i]);
  }

  // 2. The map publish frequency cannot be 0 (to avoid a divde-by-zero)
  if (map_publish_frequency_ > 0) {
    publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
  } else {
    publish_cycle_ = rclcpp::Duration(-1s);
  }

  // 3. If the footprint has been specified, it must be in the correct format
  use_radius_ = true;

  if (footprint_ != "" && footprint_ != "[]") {
    // Footprint parameter has been specified, try to convert it
    std::vector<geometry_msgs::msg::Point> new_footprint;
    if (makeFootprintFromString(footprint_, new_footprint)) {
      // The specified footprint is valid, so we'll use that instead of the radius
      use_radius_ = false;
    } else {
      // Footprint provided but invalid, so stay with the radius
      RCLCPP_ERROR(
        this->get_logger(),
        "The footprint parameter is invalid: \"%s\", using radius (%lf) instead",
        footprint_.c_str(), robot_radius_);
    }
  }
}

void Costmap2DROS::setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);
  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::setRobotFootprintPolygon(const geometry_msgs::msg::Polygon::SharedPtr footprint)
{
  setRobotFootprint(toPointVector(footprint));
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  geometry_msgs::msg::PoseStamped global_pose;
  if (!getRobotPose(global_pose)) {
    return;
  }

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(
    global_pose.pose.position.x, global_pose.pose.position.y, yaw, padded_footprint_,
    oriented_footprint);
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  RCLCPP_DEBUG(this->get_logger(), "mapUpdateLoop frequency: %lf", frequency);

  // the user might not want to run the loop every cycle
  if (frequency == 0.0) {
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Entering loop");

  rclcpp::WallRate r(frequency);  // 200ms by default

  while (rclcpp::ok() && !map_update_thread_shutdown_) {
    // Execute after start() will complete plugins activation
    if (!stopped_) {
      // Lock while modifying layered costmap and publishing values
      std::scoped_lock<std::mutex> lock(_dynamic_parameter_mutex);

      // Measure the execution time of the updateMap method
      const auto t_start = std::chrono::steady_clock::now();
      updateMap();
      const auto t_end = std::chrono::steady_clock::now();
      const double elapsed_s = std::chrono::duration<double>(t_end - t_start).count();

      RCLCPP_DEBUG(this->get_logger(), "Map update time: %.9f", elapsed_s);
      if (publish_cycle_ > rclcpp::Duration(0s) && layered_costmap_->isInitialized()) {
        unsigned int x0, y0, xn, yn;
        layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
        costmap_publisher_->updateBounds(x0, xn, y0, yn);

        auto current_time = now();
        if (
          (last_publish_ + publish_cycle_ < current_time) ||  // publish_cycle_ is due
          (current_time <
           last_publish_))  // time has moved backwards, probably due to a switch to sim_time // NOLINT
        {
          RCLCPP_DEBUG(this->get_logger(), "Publish costmap at %s", name_.c_str());
          costmap_publisher_->publishCostmap();
          last_publish_ = current_time;
        }
      }
    }

    // Make sure to sleep for the remainder of our cycle time
    r.sleep();

#if 0
    // TODO(bpwilcox): find ROS2 equivalent or port for r.cycletime()
    if (r.period() > tf2::durationFromSec(1 / frequency)) {
      RCLCPP_WARN(
        get_logger(),
        "Costmap2DROS: Map update loop missed its desired rate of %.4fHz... "
        "the loop actually took %.4f seconds", frequency, r.period());
    }
#endif
  }
}

void Costmap2DROS::updateMap()
{
  RCLCPP_DEBUG(this->get_logger(), "Updating map...");

  if (stop_updates_) {
    return;
  }

  // get global pose
  geometry_msgs::msg::PoseStamped pose;
  if (getRobotPose(pose)) {
    const double & x = pose.pose.position.x;
    const double & y = pose.pose.position.y;
    const double yaw = tf2::getYaw(pose.pose.orientation);
    layered_costmap_->updateMap(x, y, yaw);

    auto footprint = std::make_unique<geometry_msgs::msg::PolygonStamped>();
    footprint->header = pose.header;
    transformFootprint(x, y, yaw, padded_footprint_, *footprint);

    RCLCPP_DEBUG(this->get_logger(), "Publishing footprint");
    footprint_pub_->publish(std::move(footprint));
    initialized_ = true;
  }
}

void Costmap2DROS::start()
{
  RCLCPP_INFO(this->get_logger(), "start");
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

  // check if we're stopped or just paused
  if (stopped_) {
    // if we're stopped we need to re-subscribe to topics
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
         plugin != plugins->end(); ++plugin) {
      (*plugin)->activate();
    }
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
         filter != filters->end(); ++filter) {
      (*filter)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(20.0);
  while (rclcpp::ok() && !initialized_) {
    RCLCPP_DEBUG(get_logger(), "Sleeping, waiting for initialized_");
    r.sleep();
  }
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  // layered_costmap_ is set only if on_configure has been called
  if (layered_costmap_) {
    std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
    std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();

    // unsubscribe from topics
    for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
         plugin != plugins->end(); ++plugin) {
      (*plugin)->deactivate();
    }
    for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
         filter != filters->end(); ++filter) {
      (*filter)->deactivate();
    }
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  rclcpp::Rate r(100.0);
  while (!initialized_) {
    r.sleep();
  }
}

void Costmap2DROS::resetLayers()
{
  Costmap2D * top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());

  // Reset each of the plugins
  std::vector<std::shared_ptr<Layer>> * plugins = layered_costmap_->getPlugins();
  std::vector<std::shared_ptr<Layer>> * filters = layered_costmap_->getFilters();
  for (std::vector<std::shared_ptr<Layer>>::iterator plugin = plugins->begin();
       plugin != plugins->end(); ++plugin) {
    (*plugin)->reset();
  }
  for (std::vector<std::shared_ptr<Layer>>::iterator filter = filters->begin();
       filter != filters->end(); ++filter) {
    (*filter)->reset();
  }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::msg::PoseStamped & global_pose)
{
  return syncai_util::getCurrentPose(
    global_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_);
}

bool Costmap2DROS::transformPoseToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & transformed_pose)
{
  if (input_pose.header.frame_id == global_frame_) {
    transformed_pose = input_pose;
    return true;
  } else {
    return syncai_util::transformPoseInTargetFrame(
      input_pose, transformed_pose, *tf_buffer_, global_frame_, transform_tolerance_);
  }
}

rcl_interfaces::msg::SetParametersResult Costmap2DROS::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  bool resize_map = false;
  std::lock_guard<std::mutex> lock_reinit(_dynamic_parameter_mutex);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "robot_radius") {
        robot_radius_ = parameter.as_double();
        // Set the footprint
        if (use_radius_) {
          setRobotFootprint(makeFootprintFromRadius(robot_radius_));
        }
      } else if (name == "footprint_padding") {
        footprint_padding_ = parameter.as_double();
        padded_footprint_ = unpadded_footprint_;
        padFootprint(padded_footprint_, footprint_padding_);
        layered_costmap_->setFootprint(padded_footprint_);
      } else if (name == "transform_tolerance") {
        transform_tolerance_ = parameter.as_double();
      } else if (name == "publish_frequency") {
        map_publish_frequency_ = parameter.as_double();
        if (map_publish_frequency_ > 0) {
          publish_cycle_ = rclcpp::Duration::from_seconds(1 / map_publish_frequency_);
        } else {
          publish_cycle_ = rclcpp::Duration(-1s);
        }
      } else if (name == "resolution") {
        resize_map = true;
        resolution_ = parameter.as_double();
      } else if (name == "origin_x") {
        resize_map = true;
        origin_x_ = parameter.as_double();
      } else if (name == "origin_y") {
        resize_map = true;
        origin_y_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_INTEGER) {
      if (name == "width") {
        if (parameter.as_int() > 0) {
          resize_map = true;
          map_width_meters_ = parameter.as_int();
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "You try to set width of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          result.successful = false;
          return result;
        }
      } else if (name == "height") {
        if (parameter.as_int() > 0) {
          resize_map = true;
          map_height_meters_ = parameter.as_int();
        } else {
          RCLCPP_ERROR(
            get_logger(),
            "You try to set height of map to be negative or zero,"
            " this isn't allowed, please give a positive value.");
          result.successful = false;
          return result;
        }
      }
    } else if (type == ParameterType::PARAMETER_STRING) {
      if (name == "footprint") {
        footprint_ = parameter.as_string();
        std::vector<geometry_msgs::msg::Point> new_footprint;
        if (makeFootprintFromString(footprint_, new_footprint)) {
          setRobotFootprint(new_footprint);
        }
      } else if (name == "robot_base_frame") {
        // First, make sure that the transform between the robot base frame
        // and the global frame is available
        std::string tf_error;
        RCLCPP_INFO(get_logger(), "Checking transform");
        if (!tf_buffer_->canTransform(
              global_frame_, parameter.as_string(), tf2::TimePointZero, tf2::durationFromSec(1.0),
              &tf_error)) {
          RCLCPP_WARN(
            get_logger(),
            "Timed out waiting for transform from %s to %s"
            " to become available, tf error: %s",
            parameter.as_string().c_str(), global_frame_.c_str(), tf_error.c_str());
          RCLCPP_WARN(
            get_logger(),
            "Rejecting robot_base_frame change to %s , leaving it to its original"
            " value of %s",
            parameter.as_string().c_str(), robot_base_frame_.c_str());
          result.successful = false;
          return result;
        }
        robot_base_frame_ = parameter.as_string();
      }
    }
  }

  if (resize_map && !layered_costmap_->isSizeLocked()) {
    layered_costmap_->resizeMap(
      (unsigned int)(map_width_meters_ / resolution_),
      (unsigned int)(map_height_meters_ / resolution_), resolution_, origin_x_, origin_y_);
    updateMap();
  }

  result.successful = true;
  return result;
}

}  // namespace syncai_costmap_2d