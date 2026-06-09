#include "syncai_amcl/amcl_node.hpp"

#include "syncai_util/geometry_utils.hpp"

namespace syncai_amcl
{
using syncai_util::geometry_utils::orientationAroundZAxis;

AmclNode::AmclNode(const rclcpp::NodeOptions & options) : Node("amcl", options)
{
  RCLCPP_INFO(this->get_logger(), "AMCL node has been initialized.");

  initParameters();
  initTransforms();
}

AmclNode::~AmclNode()
{
}

void AmclNode::initParameters()
{
  double save_pose_rate;
  double tmp_tol;

  this->declare_parameter<double>("alpha1", rclcpp::ParameterValue(0.2));
  this->get_parameter("alpha1", alpha1_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] alpha1: %f", __func__, alpha1_);

  this->declare_parameter<double>("alpha2", rclcpp::ParameterValue(0.2));
  this->get_parameter("alpha2", alpha2_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] alpha2: %f", __func__, alpha2_);

  this->declare_parameter<double>("alpha3", rclcpp::ParameterValue(0.2));
  this->get_parameter("alpha3", alpha3_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] alpha3: %f", __func__, alpha3_);

  this->declare_parameter<double>("alpha4", rclcpp::ParameterValue(0.2));
  this->get_parameter("alpha4", alpha4_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] alpha4: %f", __func__, alpha4_);

  this->declare_parameter<double>("alpha5", rclcpp::ParameterValue(0.2));
  this->get_parameter("alpha5", alpha5_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] alpha5: %f", __func__, alpha5_);

  this->declare_parameter<double>("beam_skip_distance", rclcpp::ParameterValue(0.5));
  this->get_parameter("beam_skip_distance", beam_skip_distance_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] beam_skip_distance: %f", __func__, beam_skip_distance_);

  this->declare_parameter<double>("beam_skip_error_threshold", rclcpp::ParameterValue(0.9));
  this->get_parameter("beam_skip_error_threshold", beam_skip_error_threshold_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] beam_skip_error_threshold: %f", __func__,
    beam_skip_error_threshold_);

  this->declare_parameter<double>("beam_skip_threshold", rclcpp::ParameterValue(0.3));
  this->get_parameter("beam_skip_threshold", beam_skip_threshold_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] beam_skip_threshold: %f", __func__, beam_skip_threshold_);

  this->declare_parameter<bool>("do_beamskip", rclcpp::ParameterValue(false));
  this->get_parameter("do_beamskip", do_beamskip_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] do_beamskip: %s", __func__, do_beamskip_ ? "true" : "false");

  this->declare_parameter<std::string>("base_frame_id", rclcpp::ParameterValue("base_footprint"));
  this->get_parameter("base_frame_id", base_frame_id_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] base_frame_id: %s", __func__, base_frame_id_.c_str());

  this->declare_parameter<std::string>("global_frame_id", rclcpp::ParameterValue("map"));
  this->get_parameter("global_frame_id", global_frame_id_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] global_frame_id: %s", __func__, global_frame_id_.c_str());

  this->declare_parameter<double>("lambda_short", rclcpp::ParameterValue(0.1));
  this->get_parameter("lambda_short", lambda_short_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] lambda_short: %f", __func__, lambda_short_);

  this->declare_parameter<double>("laser_likelihood_max_dist", rclcpp::ParameterValue(2.0));
  this->get_parameter("laser_likelihood_max_dist", laser_likelihood_max_dist_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] laser_likelihood_max_dist: %f", __func__,
    laser_likelihood_max_dist_);

  this->declare_parameter<double>("laser_max_range", rclcpp::ParameterValue(100.0));
  this->get_parameter("laser_max_range", laser_max_range_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] laser_max_range: %f", __func__, laser_max_range_);

  this->declare_parameter<double>("laser_min_range", rclcpp::ParameterValue(-1.0));
  this->get_parameter("laser_min_range", laser_min_range_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] laser_min_range: %f", __func__, laser_min_range_);

  this->declare_parameter<std::string>(
    "laser_model_type", rclcpp::ParameterValue(std::string("likelihood_field")));
  this->get_parameter("laser_model_type", sensor_model_type_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] laser_model_type: %s", __func__, sensor_model_type_.c_str());

  this->declare_parameter<bool>("set_initial_pose", rclcpp::ParameterValue(false));
  this->get_parameter("set_initial_pose", set_initial_pose_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] set_initial_pose: %s", __func__,
    set_initial_pose_ ? "true" : "false");

  this->declare_parameter<double>("initial_pose_x", rclcpp::ParameterValue(0.0));
  this->get_parameter("initial_pose_x", initial_pose_x_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] initial_pose_x: %f", __func__, initial_pose_x_);

  this->declare_parameter<double>("initial_pose_y", rclcpp::ParameterValue(0.0));
  this->get_parameter("initial_pose_y", initial_pose_y_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] initial_pose_y: %f", __func__, initial_pose_y_);

  this->declare_parameter<double>("initial_pose_z", rclcpp::ParameterValue(0.0));
  this->get_parameter("initial_pose_z", initial_pose_z_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] initial_pose_z: %f", __func__, initial_pose_z_);

  this->declare_parameter<double>("initial_pose_yaw", rclcpp::ParameterValue(0.0));
  this->get_parameter("initial_pose_yaw", initial_pose_yaw_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] initial_pose_yaw: %f", __func__, initial_pose_yaw_);

  this->declare_parameter<int>("max_beams", rclcpp::ParameterValue(60));
  this->get_parameter("max_beams", max_beams_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] max_beams: %d", __func__, max_beams_);

  this->declare_parameter<int>("max_particles", rclcpp::ParameterValue(2000));
  this->get_parameter("max_particles", max_particles_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] max_particles: %d", __func__, max_particles_);

  this->declare_parameter<int>("min_particles", rclcpp::ParameterValue(500));
  this->get_parameter("min_particles", min_particles_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] min_particles: %d", __func__, min_particles_);

  this->declare_parameter<std::string>("odom_frame_id", rclcpp::ParameterValue("odom"));
  this->get_parameter("odom_frame_id", odom_frame_id_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] odom_frame_id: %s", __func__, odom_frame_id_.c_str());

  this->declare_parameter<double>("pf_err", rclcpp::ParameterValue(0.05));
  this->get_parameter("pf_err", pf_err_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] pf_err: %f", __func__, pf_err_);

  this->declare_parameter<double>("pf_z", rclcpp::ParameterValue(0.99));
  this->get_parameter("pf_z", pf_z_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] pf_z: %f", __func__, pf_z_);

  this->declare_parameter<double>("recovery_alpha_fast", rclcpp::ParameterValue(0.0));
  this->get_parameter("recovery_alpha_fast", alpha_fast_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] recovery_alpha_fast: %f", __func__, alpha_fast_);

  this->declare_parameter<double>("recovery_alpha_slow", rclcpp::ParameterValue(0.0));
  this->get_parameter("recovery_alpha_slow", alpha_slow_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] recovery_alpha_slow: %f", __func__, alpha_slow_);

  this->declare_parameter<int>("resample_interval", rclcpp::ParameterValue(1));
  this->get_parameter("resample_interval", resample_interval_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] resample_interval: %d", __func__, resample_interval_);

  this->declare_parameter<std::string>(
    "robot_model_type", rclcpp::ParameterValue(std::string("differential")));
  this->get_parameter("robot_model_type", robot_model_type_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] robot_model_type: %s", __func__, robot_model_type_.c_str());

  this->declare_parameter<double>("save_pose_rate", rclcpp::ParameterValue(0.5));
  this->get_parameter("save_pose_rate", save_pose_rate);
  save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] save_pose_rate: %f", __func__, save_pose_rate);

  this->declare_parameter<double>("sigma_hit", rclcpp::ParameterValue(0.2));
  this->get_parameter("sigma_hit", sigma_hit_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] sigma_hit: %f", __func__, sigma_hit_);

  this->declare_parameter<bool>("tf_broadcast", rclcpp::ParameterValue(true));
  this->get_parameter("tf_broadcast", tf_broadcast_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] tf_broadcast: %s", __func__, tf_broadcast_ ? "true" : "false");

  this->declare_parameter<double>("transform_tolerance", rclcpp::ParameterValue(1.0));
  this->get_parameter("transform_tolerance", tmp_tol);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] transform_tolerance: %f", __func__, tmp_tol);

  this->declare_parameter<double>("update_min_a", rclcpp::ParameterValue(0.2));
  this->get_parameter("update_min_a", a_thresh_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] update_min_a: %f", __func__, a_thresh_);

  this->declare_parameter<double>("update_min_d", rclcpp::ParameterValue(0.25));
  this->get_parameter("update_min_d", d_thresh_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] update_min_d: %f", __func__, d_thresh_);

  this->declare_parameter<double>("z_hit", rclcpp::ParameterValue(0.5));
  this->get_parameter("z_hit", z_hit_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] z_hit: %f", __func__, z_hit_);

  this->declare_parameter<double>("z_max", rclcpp::ParameterValue(0.05));
  this->get_parameter("z_max", z_max_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] z_max: %f", __func__, z_max_);

  this->declare_parameter<double>("z_rand", rclcpp::ParameterValue(0.5));
  this->get_parameter("z_rand", z_rand_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] z_rand: %f", __func__, z_rand_);

  this->declare_parameter<double>("z_short", rclcpp::ParameterValue(0.05));
  this->get_parameter("z_short", z_short_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] z_short: %f", __func__, z_short_);

  this->declare_parameter<bool>("always_reset_initial_pose", rclcpp::ParameterValue(false));
  this->get_parameter("always_reset_initial_pose", always_reset_initial_pose_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] always_reset_initial_pose: %s", __func__,
    always_reset_initial_pose_ ? "true" : "false");

  this->declare_parameter<std::string>("scan_topic", rclcpp::ParameterValue("scan"));
  this->get_parameter("scan_topic", scan_topic_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] scan_topic: %s", __func__, scan_topic_.c_str());

  this->declare_parameter<std::string>("map_topic", rclcpp::ParameterValue("map"));
  this->get_parameter("map_topic", map_topic_);
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] map_topic: %s", __func__, map_topic_.c_str());

  this->declare_parameter<bool>("first_map_only", rclcpp::ParameterValue(false));
  this->get_parameter("first_map_only", first_map_only_);
  RCLCPP_INFO(
    this->get_logger(), "[AMCL][%s] first_map_only: %s", __func__,
    first_map_only_ ? "true" : "false");
}

void AmclNode::initTransforms()
{
  RCLCPP_INFO(this->get_logger(), "[AMCL][%s] Initializing transforms", __func__);

  // Initialize transform listener and broadcaster
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
}
}  // namespace syncai_amcl