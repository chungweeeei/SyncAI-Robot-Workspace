#ifndef SYNCAI_COSTMAP_2D__COSTMAP_2D_ROS_HPP_
#define SYNCAI_COSTMAP_2D__COSTMAP_2D_ROS_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d_publisher.hpp"
#include "syncai_costmap_2d/footprint.hpp"
#include "syncai_costmap_2d/layer.hpp"
#include "syncai_costmap_2d/layered_costmap.hpp"
#include "syncai_util/node_thread.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace syncai_costmap_2d
{
class ClearCostmapService;

class Costmap2DROS : public rclcpp::Node
{
public:
  /**
   * @brief  Constructor for the wrapper
   * @param options Additional options to control creation of the node.
   */
  Costmap2DROS(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief  Constructor for the wrapper, the node will be placed in a namespace equal to the node's name
   * @param name Name of the costmap ROS node
   */
  explicit Costmap2DROS(const std::string & name);

  /**
   * @brief Constructor for the wrapper, with explicit parent / local namespaces
   * @param name Name of the costmap ROS node
   * @param parent_namespace Namespace of the parent node (e.g. "/")
   * @param local_namespace Local namespace for this costmap
   */
  Costmap2DROS(
    const std::string & name, const std::string & parent_namespace,
    const std::string & local_namespace);

  /**
   * @brief A destructor
   */
  ~Costmap2DROS();

  /**
   * @brief Initialize the costmap after construction (manual on_configure).
   *
   * Must be called once after the node is held by a std::shared_ptr (i.e.
   * after std::make_shared<Costmap2DROS>()), since it relies on
   * shared_from_this() for parameter declaration, the tf listener and
   * plugin initialization. Reads parameters, creates the tf buffer/listener,
   * builds the LayeredCostmap, loads and initializes the configured layer
   * plugins, sets up the publishers and starts the map update thread.
   */
  void init();

  /**
   * @brief Activate the costmap (manual on_activate).
   *
   * Call once after init(). Waits for the robot transform, spawns the map
   * update thread, activates the layer plugins and registers the dynamic
   * parameter callback.
   */
  void activate();

  /**
   * @brief Deactivate the costmap (manual on_deactivate).
   *
   * Reverses activate(): unregisters the dynamic parameter callback, stops the
   * layer plugins and joins the map update thread. Safe to call from the
   * destructor.
   */
  void deactivate();

  /**
   * @brief  Subscribes to sensor topics if necessary and starts costmap
   * updates, can be called to restart the costmap after calls to either
   * stop() or pause()
   */
  void start();

  /**
   * @brief  Stops costmap updates and unsubscribes from sensor topics
   */
  void stop();

  /**
   * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
   */
  void pause();

  /**
   * @brief  Resumes costmap updates
   */
  void resume();

  /**
   * @brief Update the map with the layered costmap / plugins
   */
  void updateMap();

  /**
   * @brief Reset each of the layers and the master costmap.
   */
  void resetLayers();

  /** @brief Same as getLayeredCostmap()->isCurrent(). */
  bool isCurrent() { return layered_costmap_->isCurrent(); }

  /**
   * @brief Get the pose of the robot in the global frame of the costmap
   * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
   * @return True if the pose was set successfully, false otherwise
   */
  bool getRobotPose(geometry_msgs::msg::PoseStamped & global_pose);

  /**
   * @brief Transform the input_pose in the global frame of the costmap
   * @param input_pose pose to be transformed
   * @param transformed_pose pose transformed
   * @return True if the pose was transformed successfully, false otherwise
   */
  bool transformPoseToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & input_pose,
    geometry_msgs::msg::PoseStamped & transformed_pose);

  /** @brief Returns costmap name */
  std::string getName() const { return name_; }

  /** @brief Returns the delay in transform (tf) data that is tolerable in seconds */
  double getTransformTolerance() const { return transform_tolerance_; }

  /**
   * @brief Return a pointer to the "master" costmap which receives updates from all the layers.
   *
   * Same as calling getLayeredCostmap()->getCostmap().
   */
  Costmap2D * getCostmap() { return layered_costmap_->getCostmap(); }

  /**
   * @brief  Returns the global frame of the costmap
   * @return The global frame of the costmap
   */
  std::string getGlobalFrameID() { return global_frame_; }

  /**
   * @brief  Returns the local frame of the costmap
   * @return The local frame of the costmap
   */
  std::string getBaseFrameID() { return robot_base_frame_; }

  /**
   * @brief Get the layered costmap object used in the node
   */
  LayeredCostmap * getLayeredCostmap() { return layered_costmap_.get(); }

  /** @brief Returns the current padded footprint as a geometry_msgs::msg::Polygon. */
  geometry_msgs::msg::Polygon getRobotFootprintPolygon()
  {
    return syncai_costmap_2d::toPolygon(padded_footprint_);
  }

  /** @brief Return the current footprint of the robot as a vector of points.
   *
   * This version of the footprint is padded by the footprint_padding_
   * distance, set in the rosparam "footprint_padding".
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::msg::Point> getRobotFootprint() { return padded_footprint_; }

  /** @brief Return the current unpadded footprint of the robot as a vector of points.
   *
   * This is the raw version of the footprint without padding.
   *
   * The footprint initially comes from the rosparam "footprint" but
   * can be overwritten by dynamic reconfigure or by messages received
   * on the "footprint" topic. */
  std::vector<geometry_msgs::msg::Point> getUnpaddedRobotFootprint() { return unpadded_footprint_; }

  /**
   * @brief  Build the oriented footprint of the robot at the robot's current pose
   * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
   */
  void getOrientedFootprint(std::vector<geometry_msgs::msg::Point> & oriented_footprint);

  /** @brief Set the footprint of the robot to be the given set of
   * points, padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setRobotFootprint(const std::vector<geometry_msgs::msg::Point> & points);

  /** @brief Set the footprint of the robot to be the given polygon,
   * padded by footprint_padding.
   *
   * Should be a convex polygon, though this is not enforced.
   *
   * First expands the given polygon by footprint_padding_ and then
   * sets padded_footprint_ and calls
   * layered_costmap_->setFootprint().  Also saves the unpadded
   * footprint, which is available from
   * getUnpaddedRobotFootprint(). */
  void setRobotFootprintPolygon(const geometry_msgs::msg::Polygon::SharedPtr footprint);

  std::shared_ptr<tf2_ros::Buffer> getTfBuffer() { return tf_buffer_; }

  /**
   * @brief  Get the costmap's use_radius_ parameter, corresponding to
   * whether the footprint for the robot is a circle with radius robot_radius_
   * or an arbitrarily defined footprint in footprint_.
   * @return  use_radius_
   */
  bool getUseRadius() { return use_radius_; }

  /**
   * @brief  Get the costmap's robot_radius_ parameter, corresponding to
   * raidus of the robot footprint when it is defined as as circle
   * (i.e. when use_radius_ == true).
   * @return  robot_radius_
   */
  double getRobotRadius() { return robot_radius_; }

protected:
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  std::unique_ptr<Costmap2DPublisher> costmap_publisher_{nullptr};

  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr footprint_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_sub_;

  // Dedicated callback group and executor for tf timer_interface and message fillter
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::unique_ptr<syncai_util::NodeThread> executor_thread_;

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<LayeredCostmap> layered_costmap_;
  std::string name_;
  std::string parent_namespace_;

  /**
   * @brief Function on timer for costmap update
   */
  void mapUpdateLoop(double frequency);
  bool map_update_thread_shutdown_{false};
  std::atomic<bool> stop_updates_{false};
  std::atomic<bool> initialized_{false};
  std::atomic<bool> stopped_{true};
  std::unique_ptr<std::thread> map_update_thread_;  ///< @brief A thread for updating the map
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration publish_cycle_{1, 0};
  pluginlib::ClassLoader<Layer> plugin_loader_{"syncai_costmap_2d", "syncai_costmap_2d::Layer"};

  /**
   * @brief Get parameters for node
   */
  void getParameters();
  bool always_send_full_costmap_{false};
  std::string footprint_;
  float footprint_padding_{0};
  std::string global_frame_;  ///< The global frame for the costmap
  int map_height_meters_{0};
  double map_publish_frequency_{0};
  double map_update_frequency_{0};
  int map_width_meters_{0};
  double origin_x_{0};
  double origin_y_{0};
  std::vector<std::string> default_plugins_;
  std::vector<std::string> default_types_;
  std::vector<std::string> plugin_names_;
  std::vector<std::string> plugin_types_;
  std::vector<std::string> filter_names_;
  std::vector<std::string> filter_types_;
  double resolution_{0};
  std::string robot_base_frame_;  ///< The frame_id of the robot base
  double robot_radius_;
  bool rolling_window_{false};  ///< Whether to use a rolling window version of the costmap
  bool track_unknown_space_{false};
  double transform_tolerance_{0};  ///< The timeout before transform errors

  // Derived parameters
  bool use_radius_{false};
  std::vector<geometry_msgs::msg::Point> unpadded_footprint_;
  std::vector<geometry_msgs::msg::Point> padded_footprint_;

  // Services to clear the costmap (entirely / around robot / except a region).
  std::unique_ptr<ClearCostmapService> clear_costmap_service_;

  // Mutex guarding map updates against concurrent dynamic-parameter callbacks.
  std::mutex _dynamic_parameter_mutex;

  // Dynamic parameters handler
  OnSetParametersCallbackHandle::SharedPtr dyn_params_handler;

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);
};
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__COSTMAP_2D_ROS_HPP_