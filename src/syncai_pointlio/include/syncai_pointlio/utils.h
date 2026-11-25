#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

class Utils
{
public:
    // static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2 &msg);
    // static sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    static double getSec(std_msgs::msg::Header &header);
    // filter_num decimates by raw point index (the cloud the filter sees). With
    // dense_out and 0 < dense_filter_num < filter_num, *dense_out gets the same
    // scan decimated by dense_filter_num instead, in the same pass; when
    // filter_num is a multiple of it, the main cloud's points are a subset.
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0, int dense_filter_num = 0, pcl::PointCloud<pcl::PointXYZINormal>::Ptr *dense_out = nullptr);
    // PointCloud2 path for sensors that publish sensor_msgs/PointCloud2 (e.g. Isaac Sim):
    // curvature (per-point time offset in ms) is set to 0 -> with nothing else, the cloud is
    // an instantaneous snapshot (no motion deskew), matching a full rendered scan.
    //
    // With time_field (2026-11-25) each kept point's absolute time, seconds, is read from that
    // float64 field into *times (and *dense_times for the dense cloud), index for index with
    // the cloud -- for a cloud that does carry it (the Lynx M20's `timestamp`), which
    // ScanMerger then cuts into scans by. A point whose time is not finite is skipped. When the
    // cloud has no such float64 field, *times comes back empty and the caller decides.
    //
    // With source_field (2026-11-25) each kept point of the main cloud also gets that integer
    // field's value in *sources -- which lidar of a merged cloud it came from (the M20's
    // `ring`: 0-95 front, 96-191 rear), for ScanMerger to wait for the late one. Empty when
    // the cloud has no such integer field.
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc2ToPCL(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range = 0.5,
      double max_range = 20.0, int dense_filter_num = 0,
      pcl::PointCloud<pcl::PointXYZINormal>::Ptr * dense_out = nullptr,
      const std::string & time_field = "", std::vector<double> * times = nullptr,
      std::vector<double> * dense_times = nullptr, const std::string & source_field = "",
      std::vector<long> * sources = nullptr);
    static builtin_interfaces::msg::Time getTime(const double& sec);
};