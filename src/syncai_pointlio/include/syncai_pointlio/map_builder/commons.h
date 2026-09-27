#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;

template <typename T>
using Vec = std::vector<T>;

bool esti_plane(PointVec & points, const double & thresh, V4D & out);

float sq_dist(const PointType & p1, const PointType & p2);

struct Config
{
  int lidar_filter_num = 3;
  double lidar_min_range = 0.5;
  double lidar_max_range = 20.0;
  double scan_resolution = 0.25;
  double map_resolution = 0.3;

  double cube_len = 300;
  double det_range = 60;
  double move_thresh = 1.5;

  // Point-LIO output model: omg/acc are random-walk states, these drive them.
  double gyr_cov_output = 1000.0;
  double acc_cov_output = 500.0;
  double b_gyr_cov = 0.0001;
  double b_acc_cov = 0.0001;
  // IMU treated as a measurement (not an input) in the output model.
  double imu_meas_omg_cov = 0.1;
  double imu_meas_acc_cov = 0.1;
  // Saturation thresholds: axes at/over these are dropped from the IMU update.
  // satu_gyro in rad/s; satu_acc in m/s^2 AFTER imu_acc_scale is applied.
  double satu_gyro = 35.0;
  double satu_acc = 30.0;

  double lidar_meas_cov = 0.01;
  double plane_thr = 0.1;
  // A same-timestamp point group larger than this is split into sequential
  // chunks (equivalent for independent noise) to bound the m x m solve.
  int batch_max_points = 500;

  int imu_init_num = 20;
  int near_search_num = 5;
  bool gravity_align = true;
  bool esti_il = false;
  M3D r_il = M3D::Identity();
  V3D t_il = V3D::Zero();
};

struct IMUData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  V3D acc;
  V3D gyro;
  double time;
  IMUData() = default;
  IMUData(const V3D & a, const V3D & g, double & t) : acc(a), gyro(g), time(t) {}
};

struct SyncPackage
{
  Vec<IMUData> imus;
  CloudType::Ptr cloud;
  double cloud_start_time = 0.0;
  double cloud_end_time = 0.0;
};
