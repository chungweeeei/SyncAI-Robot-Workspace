#include "scan_merger.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

ScanMerger::ScanMerger() : Node("ros2_laser_scan_merger")
{
  initialize_params();

  laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
  laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();

  // register laser scan subscribers & pointcloud publisher
  auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    topic1_, default_qos, std::bind(&ScanMerger::scan_callback1, this, std::placeholders::_1));
  sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    topic2_, default_qos, std::bind(&ScanMerger::scan_callback2, this, std::placeholders::_1));

  point_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS());
}

void ScanMerger::scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser1_ = msg;
  update_point_cloud();
}

void ScanMerger::scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  /**
   * @brief scan2 callback function only update the laser2_ variable
   */
  laser2_ = msg;
}

void ScanMerger::update_point_cloud()
{
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
  int count = 0;
  if (show1_ && laser1_) {
    float temp_min_, temp_max_;
    if (laser1_->angle_min < laser1_->angle_max) {
      temp_min_ = laser1_->angle_min;
      temp_max_ = laser1_->angle_max;
    } else {
      temp_min_ = laser1_->angle_max;
      temp_max_ = laser1_->angle_min;
    }

    for (float i = temp_min_; i <= temp_max_ && count < laser1_->ranges.size();
         i += laser1_->angle_increment) {
      pcl::PointXYZRGB pt;
      pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
      int used_count_ = count;
      if (flip1_) {
        used_count_ = (int)laser1_->ranges.size() - 1 - count;
      }
      // SyncAI patch: skip invalid returns. Isaac Sim reports -1.0 for
      // no-return beams; left unfiltered the merger plots them at radius
      // |range| (a 1 m phantom ring). Honor the LaserScan range bounds.
      float range1_ = laser1_->ranges[used_count_];
      if (!std::isfinite(range1_) || range1_ < laser1_->range_min || range1_ > laser1_->range_max) {
        count++;
        continue;
      }
      // 把 scan range 轉成 (x, y) 座標
      float temp_x = range1_ * std::cos(i);
      float temp_y = range1_ * std::sin(i);
      pt.x = temp_x * std::cos(laser1Alpha_ * M_PI / 180) -
             temp_y * std::sin(laser1Alpha_ * M_PI / 180) + laser1XOff_;
      pt.y = temp_x * std::sin(laser1Alpha_ * M_PI / 180) +
             temp_y * std::cos(laser1Alpha_ * M_PI / 180) + laser1YOff_;
      pt.z = laser1ZOff_;

      // ┌────────────────────────────┬─────────────────────┬──────────────┐
      // │     beam 角度 i 的位置       │ inverse=false(預設) │ inverse=true │
      // ├────────────────────────────┼─────────────────────┼──────────────┤
      // │ 在 [angleMin, angleMax] 內  │ ✅ 保留             │ ❌ 丟棄        │
      // ├────────────────────────────┼─────────────────────┼──────────────┤
      // │ 在 [angleMin, angleMax] 外  │ ❌ 丟棄             │ ✅ 保留        │
      // └────────────────────────────┴─────────────────────┴──────────────┘

      if ((i < (laser1AngleMin_ * M_PI / 180)) || (i > (laser1AngleMax_ * M_PI / 180))) {
        if (inverse1_) {
          cloud_.points.push_back(pt);
        }
      } else {
        if (!inverse1_) {
          cloud_.points.push_back(pt);
        }
      }
      count++;
    }
  }

  count = 0;
  if (show2_ && laser2_) {
    float temp_min_, temp_max_;
    if (laser2_->angle_min < laser2_->angle_max) {
      temp_min_ = laser2_->angle_min;
      temp_max_ = laser2_->angle_max;
    } else {
      temp_min_ = laser2_->angle_max;
      temp_max_ = laser2_->angle_min;
    }
    for (float i = temp_min_; i <= temp_max_ && count < laser2_->ranges.size();
         i += laser2_->angle_increment) {
      pcl::PointXYZRGB pt;
      pt = pcl::PointXYZRGB(laser2R_, laser2G_, laser2B_);

      int used_count_ = count;
      if (flip2_) {
        used_count_ = (int)laser2_->ranges.size() - 1 - count;
      }

      // SyncAI patch: skip invalid returns (see laser1 above).
      float range2_ = laser2_->ranges[used_count_];
      if (!std::isfinite(range2_) || range2_ < laser2_->range_min || range2_ > laser2_->range_max) {
        count++;
        continue;
      }
      float temp_x = range2_ * std::cos(i);
      float temp_y = range2_ * std::sin(i);
      pt.x = temp_x * std::cos(laser2Alpha_ * M_PI / 180) -
             temp_y * std::sin(laser2Alpha_ * M_PI / 180) + laser2XOff_;
      pt.y = temp_x * std::sin(laser2Alpha_ * M_PI / 180) +
             temp_y * std::cos(laser2Alpha_ * M_PI / 180) + laser2YOff_;
      pt.z = laser2ZOff_;
      if ((i < (laser2AngleMin_ * M_PI / 180)) || (i > (laser2AngleMax_ * M_PI / 180))) {
        if (inverse2_) {
          cloud_.points.push_back(pt);
        }
      } else {
        if (!inverse2_) {
          cloud_.points.push_back(pt);
        }
      }
      count++;
    }
  }

  auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud_, *pc2_msg_);
  pc2_msg_->header.frame_id = cloudFrameId_;
  pc2_msg_->header.stamp = now();
  if (show1_ && laser1_) {
    pc2_msg_->header.stamp = laser1_->header.stamp;
  }
  if (show2_ && laser2_) {
    pc2_msg_->header.stamp = laser2_->header.stamp;
  }
  pc2_msg_->is_dense = false;
  point_cloud_pub_->publish(*pc2_msg_);
}

void ScanMerger::initialize_params()
{
  this->declare_parameter("pointCloudTopic", "base/custom_cloud");
  this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "pointCloud");
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] pointCloudTopic: %s", __func__, cloudTopic_.c_str());

  this->declare_parameter("pointCloudFrameId", "laser");
  this->get_parameter_or<std::string>("pointCloudFrameId", cloudFrameId_, "laser");
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] pointCloudFrameId: %s", __func__, cloudFrameId_.c_str());

  // scan1 parameters
  this->declare_parameter("scanTopic1", "lidar_front_right/scan");
  this->get_parameter_or<std::string>("scanTopic1", topic1_, "lidar_front_right/scan");
  RCLCPP_INFO(this->get_logger(), "[ScanMerger][%s] scan1 topic: %s", __func__, topic1_.c_str());

  this->declare_parameter("laser1XOff", -0.45);
  this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
  this->declare_parameter("laser1YOff", 0.24);
  this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
  this->declare_parameter("laser1ZOff", 0.0);
  this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
  this->declare_parameter("laser1Alpha", 45.0);
  this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
  this->declare_parameter("laser1AngleMin", -181.0);
  this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, -181.0);
  this->declare_parameter("laser1AngleMax", 181.0);
  this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 181.0);
  this->declare_parameter("laser1R", 255);
  this->get_parameter_or<uint8_t>("laser1R", laser1R_, 0);
  this->declare_parameter("laser1G", 0);
  this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
  this->declare_parameter("laser1B", 0);
  this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] laser1 offset: [%f, %f, %f], alpha: %f", __func__,
    laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_);
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] laser1 angle range: [%f, %f]", __func__, laser1AngleMin_,
    laser1AngleMax_);
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] laser1 color: [%d, %d, %d]", __func__, laser1R_, laser1G_,
    laser1B_);

  this->declare_parameter("show1", true);
  this->get_parameter_or<bool>("show1", show1_, true);
  this->declare_parameter("flip1", false);
  this->get_parameter_or<bool>("flip1", flip1_, false);
  this->declare_parameter("inverse1", false);
  this->get_parameter_or<bool>("inverse1", inverse1_, false);

  this->declare_parameter("scanTopic2", "lidar_rear_left/scan");
  this->get_parameter_or<std::string>("scanTopic2", topic2_, "lidar_rear_left/scan");
  RCLCPP_INFO(this->get_logger(), "[ScanMerger][%s] scan2 topic: %s", __func__, topic2_.c_str());

  this->declare_parameter("laser2XOff", 0.315);
  this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
  this->declare_parameter("laser2YOff", -0.24);
  this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
  this->declare_parameter("laser2ZOff", 0.0);
  this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
  this->declare_parameter("laser2Alpha", 225.0);
  this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
  this->declare_parameter("laser2AngleMin", -181.0);
  this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, -181.0);
  this->declare_parameter("laser2AngleMax", 181.0);
  this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 181.0);
  this->declare_parameter("laser2R", 0);
  this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
  this->declare_parameter("laser2G", 0);
  this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
  this->declare_parameter("laser2B", 255);
  this->get_parameter_or<uint8_t>("laser2B", laser2B_, 0);

  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] laser2 offset: [%f, %f, %f], alpha: %f", __func__,
    laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_);
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] laser2 angle range: [%f, %f]", __func__, laser2AngleMin_,
    laser2AngleMax_);
  RCLCPP_INFO(
    this->get_logger(), "[ScanMerger][%s] laser2 color: [%d, %d, %d]", __func__, laser2R_, laser2G_,
    laser2B_);

  this->declare_parameter("show2", true);
  this->get_parameter_or<bool>("show2", show2_, false);
  this->declare_parameter("flip2", false);
  this->get_parameter_or<bool>("flip2", flip2_, false);
  this->declare_parameter("inverse2", false);
  this->get_parameter_or<bool>("inverse2", inverse2_, false);
}