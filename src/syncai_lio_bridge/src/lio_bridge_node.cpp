// LIO odometry provider — the robot's only odometry source (C++ port).
//
// Wheel odometry is gone (the Isaac Sim OmniGraph odom publishers are
// disabled): the robot relies solely on Point-LIO lidar-inertial odometry.
// This node turns the LIO chain (map -> lio_odom -> lio_body) into the planar
// chain the nav stack consumes:
//
//     odom -> base_link   from /<robot_id>/pointlio/lio_odom, projected to 2D
//                         (lio_body is physically <robot_id>/lidar_top, so the
//                         static base_link->lidar_top extrinsic maps it to
//                         base)
//     /<robot_id>/odom    nav_msgs/Odometry republished for twist consumers
//                         (controller_server, task_runner, robot_state); linear
//                         velocity comes from LIO (body frame), angular.z from
//                         the lidar IMU gyro since LIO leaves twist.angular
//                         empty
//     map -> odom         AMCL-style correction using the localizer's
//                         map -> lio_odom TF:
//                         map->odom = P2D(map->base) * inv(P2D(odom->base))
//
// Everything is projected to 2D (x, y, yaw; z/roll/pitch zeroed) before
// broadcasting so the planar nav stack never sees a tilted frame.
//
// This is a straight port of the original Python node; the numpy 4x4
// homogeneous-matrix math is replaced with tf2::Transform to avoid the Python
// per-tick CPU cost of numpy allocations and the tf2 Python bindings.

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace
{

// Planar (x, y, yaw) projection of a rigid transform. Matches the Python
// project_2d(): yaw = atan2(R[1][0], R[0][0]), translation x/y taken directly.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

Pose2D project_2d(const tf2::Transform & t)
{
  const tf2::Matrix3x3 & b = t.getBasis();
  const tf2::Vector3 & o = t.getOrigin();
  return Pose2D{o.x(), o.y(), std::atan2(b[1][0], b[0][0])};
}

// Planar (x, y, yaw) -> rigid transform (z/roll/pitch zero).
tf2::Transform pose_2d_transform(const Pose2D & p)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, p.yaw);
  return tf2::Transform(q, tf2::Vector3(p.x, p.y, 0.0));
}

// geometry_msgs translation + rotation -> tf2::Transform (rigid).
tf2::Transform to_transform(
  const geometry_msgs::msg::Vector3 & translation, const geometry_msgs::msg::Quaternion & rotation)
{
  tf2::Quaternion q(rotation.x, rotation.y, rotation.z, rotation.w);
  return tf2::Transform(q, tf2::Vector3(translation.x, translation.y, translation.z));
}

tf2::Transform pose_to_transform(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  return tf2::Transform(q, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

}  // namespace

class LioBridgeNode : public rclcpp::Node
{
public:
  LioBridgeNode() : rclcpp::Node("lio_bridge_node")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_top");
    const double rate = declare_parameter<double>("publish_rate", 20.0);
    // Future-date the stamp (like AMCL's transform_tolerance) so consumers can
    // interpolate between our broadcasts without extrapolation errors. Also
    // bridges the gap between the ~10 Hz LIO updates and this node's rate.
    transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Sensor-data QoS on both subs: best-effort is compatible with either
    // reliable (lio_node) or best-effort publishers.
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "pointlio/lio_odom", rclcpp::SensorDataQoS(),
      std::bind(&LioBridgeNode::lio_cb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "livox/imu", rclcpp::SensorDataQoS(),
      std::bind(&LioBridgeNode::imu_cb, this, std::placeholders::_1));
    // Reliable publisher: the SensorDataQoS (best-effort) subscribers of
    // robot_state and the reliable nav odom subscribers both match it.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    const auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LioBridgeNode::timer_cb, this));

    RCLCPP_INFO(
      get_logger(),
      "lio_bridge: LIO odometry provider — %s -> %s + odom topic from "
      "pointlio/lio_odom, %s -> %s from the localizer TF @ %.1f Hz",
      odom_frame_.c_str(), base_frame_.c_str(), map_frame_.c_str(), odom_frame_.c_str(), rate);
  }

private:
  void lio_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    lio_pose_ = pose_to_transform(msg->pose.pose);
    lio_odom_frame_ = msg->header.frame_id;
    lio_linear_x_ = msg->twist.twist.linear.x;
    lio_linear_y_ = msg->twist.twist.linear.y;
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // IMU is co-located with the lidar; planar robot -> gyro z is yaw rate.
    yaw_rate_ = msg->angular_velocity.z;
  }

  // Cached static base_link -> lidar_top mount extrinsic.
  bool base_lidar(tf2::Transform & out)
  {
    if (!base_lidar_) {
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000, "waiting for %s -> %s: %s", base_frame_.c_str(),
          lidar_frame_.c_str(), ex.what());
        return false;
      }
      base_lidar_ = to_transform(t.transform.translation, t.transform.rotation);
    }
    out = *base_lidar_;
    return true;
  }

  geometry_msgs::msg::TransformStamped make_tf(
    const rclcpp::Time & stamp, const std::string & frame_id, const std::string & child_frame_id,
    const Pose2D & p) const
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.transform.translation.x = p.x;
    msg.transform.translation.y = p.y;
    msg.transform.translation.z = 0.0;
    msg.transform.rotation.z = std::sin(p.yaw / 2.0);
    msg.transform.rotation.w = std::cos(p.yaw / 2.0);
    return msg;
  }

  void timer_cb()
  {
    if (!lio_pose_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for pointlio/lio_odom (LIO initializing?)");
      return;
    }
    tf2::Transform m_base_lidar;
    if (!base_lidar(m_base_lidar)) {
      return;
    }

    // lio_body is physically the lidar frame, so
    // lio_odom->base = lio_odom->lio_body * lidar->base.
    const tf2::Transform m_lioodom_base = *lio_pose_ * m_base_lidar.inverse();
    const Pose2D ob = project_2d(m_lioodom_base);

    const rclcpp::Time now = this->now();
    const rclcpp::Time tf_stamp = now + rclcpp::Duration::from_seconds(transform_tolerance_);

    // odom -> base_link: published before relocalization, like AMCL where the
    // odom chain exists before an initial pose.
    tf_broadcaster_->sendTransform(make_tf(tf_stamp, odom_frame_, base_frame_, ob));

    // Odometry topic for the twist consumers (controller_server, task_runner,
    // robot_state — none of them read the pose).
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = ob.x;
    odom.pose.pose.position.y = ob.y;
    odom.pose.pose.orientation.z = std::sin(ob.yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(ob.yaw / 2.0);
    odom.twist.twist.linear.x = lio_linear_x_;
    odom.twist.twist.linear.y = lio_linear_y_;
    odom.twist.twist.angular.z = yaw_rate_;
    odom_pub_->publish(odom);

    // map -> odom: needs the localizer's map -> lio_odom, which only exists
    // after /localizer/relocalize has been given a map.
    geometry_msgs::msg::TransformStamped t_map_lioodom;
    try {
      t_map_lioodom = tf_buffer_->lookupTransform(map_frame_, lio_odom_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "waiting for TF (relocalized yet?): %s", ex.what());
      return;
    }

    // Project map->base to 2D, then subtract the planar odom->base pose; both
    // are planar so the composition stays planar and consistent.
    const tf2::Transform m_map_base =
      to_transform(t_map_lioodom.transform.translation, t_map_lioodom.transform.rotation) *
      m_lioodom_base;
    const Pose2D mb = project_2d(m_map_base);
    const tf2::Transform m_map_odom = pose_2d_transform(mb) * pose_2d_transform(ob).inverse();
    const Pose2D mo = project_2d(m_map_odom);

    tf_broadcaster_->sendTransform(make_tf(tf_stamp, map_frame_, odom_frame_, mo));

    if (!localized_) {
      localized_ = true;
      RCLCPP_INFO(
        get_logger(), "localization bridged: %s -> %s = (%.3f, %.3f, yaw %.3f)", map_frame_.c_str(),
        odom_frame_.c_str(), mo.x, mo.y, mo.yaw);
    }
  }

  // Parameters.
  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;
  double transform_tolerance_{0.1};

  // TF.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ROS I/O.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest LIO state (set in callbacks, consumed by the timer).
  std::optional<tf2::Transform> lio_pose_;    // lio_odom -> lio_body
  std::string lio_odom_frame_;                // from the msg header
  double lio_linear_x_{0.0};                  // body-frame linear velocity x
  double lio_linear_y_{0.0};                  // body-frame linear velocity y
  double yaw_rate_{0.0};                      // from the lidar IMU gyro
  std::optional<tf2::Transform> base_lidar_;  // base_link -> lidar_top (cached)

  bool localized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
