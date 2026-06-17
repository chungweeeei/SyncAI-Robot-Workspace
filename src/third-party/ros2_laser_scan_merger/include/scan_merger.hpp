#ifndef SYNCAI_SCAN_MERGER_HPP_
#define SYNCAI_SCAN_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

class ScanMerger : public rclcpp::Node
{
public:
  ScanMerger();
  ~ScanMerger() = default;

private:
  void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void update_point_cloud();

  void initialize_params();

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

#endif  // SYNCAI_SCAN_MERGER_HPP_
