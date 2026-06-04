//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#ifndef ROS2_LASER_SCAN_MERGER__LASER_SCAN_MERGER_HPP_
#define ROS2_LASER_SCAN_MERGER__LASER_SCAN_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class scanMerger : public rclcpp::Node
{
public:
  scanMerger();

private:
  // initialized parameters
  void init_parameters();
  void init_publisher();
  void init_subscriber();

  // callback functions
  void scan1_cb(const sensor_msgs::msg::LaserScan::SharedPtr _msg);
  void scan2_cb(const sensor_msgs::msg::LaserScan::SharedPtr _msg);

  void update_point_cloud_rgb();
  float GET_R(float x, float y);
  float GET_THETA(float x, float y);
  float interpolate(
    float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle);

  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;

  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;

  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  uint8_t laser2R_, laser2G_, laser2B_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  sensor_msgs::msg::LaserScan::SharedPtr laser1_;
  sensor_msgs::msg::LaserScan::SharedPtr laser2_;
};

#endif  // ROS2_LASER_SCAN_MERGER__LASER_SCAN_MERGER_HPP_
