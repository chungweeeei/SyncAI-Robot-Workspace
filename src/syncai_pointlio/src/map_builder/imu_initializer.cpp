#include "syncai_pointlio/map_builder/imu_initializer.h"

IMUInitializer::IMUInitializer(Config & config, std::shared_ptr<PointEKF> kf)
: m_config(config), m_kf(kf)
{
  m_imu_cache.clear();
}

bool IMUInitializer::initialize(SyncPackage & package)
{
  // Append the IMU data collected in the package to the cache
  m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

  // Whatever rate the IMU runs at, at least N samples are averaged: if the rate is low, enough
  // data is still needed for the average to be meaningful
  if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num)) return false;

  // Average the acceleration and angular rate
  V3D acc_mean = V3D::Zero();
  V3D gyro_mean = V3D::Zero();
  for (const auto & imu : m_imu_cache) {
    acc_mean += imu.acc;
    gyro_mean += imu.gyro;
  }
  acc_mean /= static_cast<double>(m_imu_cache.size());
  gyro_mean /= static_cast<double>(m_imu_cache.size());

  // Set the extrinsic, i.e. the rigid-body transform between the IMU and the LiDAR
  m_kf->x().r_il = m_config.r_il;  // lidar -> IMU rotation (r_il is i <- l, see pointlio.yaml)
  m_kf->x().t_il = m_config.t_il;  // lidar -> IMU translation
  m_kf->x().bg = gyro_mean;
  if (m_config.gravity_align) {
    m_kf->x().r_wi =
      (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
    m_kf->x().initGravityDir(V3D(0, 0, -1.0));
  } else
    m_kf->x().initGravityDir(-acc_mean);

  // The output model's omg/acc states: at rest omg ~ 0, acc (specific force) = -R^T g
  m_kf->x().omg = gyro_mean - m_kf->x().bg;
  m_kf->x().acc = -m_kf->x().r_wi.transpose() * m_kf->x().g;

  // P() -> error state covariance matrix
  m_kf->P().setIdentity();
  m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
  m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
  m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
  m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;
  m_kf->P().block<3, 3>(21, 21) = M3D::Identity() * 0.0001;
  // omg & acc: the two blocks below are the states pointlio adds
  m_kf->P().block<3, 3>(24, 24) = M3D::Identity() * 0.01;
  m_kf->P().block<3, 3>(27, 27) = M3D::Identity() * 0.01;
  return true;
}
