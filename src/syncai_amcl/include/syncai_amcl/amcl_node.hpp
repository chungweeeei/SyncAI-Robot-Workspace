#ifndef SYNCAI_AMCL__AMCL_NODE_HPP_
#define SYNCAI_AMCL__AMCL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace syncai_amcl
{
class AmclNode : public rclcpp::Node
{
public:
  /*
   * @brief AMCL constructor
   * @param options Additional options to control creation of the node.
   */
  explicit AmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /*
   * @brief AMCL destructor
   */
  ~AmclNode();

private:
  void initParameters();
  void initTransforms();

  // Dedicated callback group and executor for services and subscriptions in AmclNode,
  // in order to isolate TF timer used in message filter.
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  bool first_map_only_{true};

  /**
   * @brief Handle a new pose estimate callback
   */
  void handleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg);
  bool init_pose_received_on_inactive{false};
  bool initial_pose_is_known_{false};
  bool set_initial_pose_{false};
  bool always_reset_initial_pose_;
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_yaw_;

  /*
   * @brief Get ROS parameters for node
   */
  double alpha1_;
  double alpha2_;
  double alpha3_;
  double alpha4_;
  double alpha5_;
  std::string base_frame_id_;
  double beam_skip_distance_;
  double beam_skip_error_threshold_;
  double beam_skip_threshold_;
  std::string global_frame_id_;
  double lambda_short_;
  double laser_likelihood_max_dist_;
  double laser_max_range_;
  double laser_min_range_;
  std::string sensor_model_type_;
  int max_beams_;
  int max_particles_;
  int min_particles_;
  std::string odom_frame_id_;
  double pf_err_;
  double pf_z_;
  double alpha_fast_;
  double alpha_slow_;
  int resample_interval_;
  std::string robot_model_type_;
  tf2::Duration save_pose_period_;
  double sigma_hit_;
  bool tf_broadcast_;
  tf2::Duration transform_tolerance_;
  double a_thresh_;
  double d_thresh_;
  double z_hit_;
  double z_max_;
  double z_short_;
  double z_rand_;
  std::string scan_topic_{"scan"};
  std::string map_topic_{"map"};
};
}  // namespace syncai_amcl

#endif