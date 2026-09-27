#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "syncai_pointlio/map_builder/commons.h"

using M12D = Eigen::Matrix<double, 12, 12>;
using M30D = Eigen::Matrix<double, 30, 30>;

using V30D = Eigen::Matrix<double, 30, 1>;
using M30X12D = Eigen::Matrix<double, 30, 12>;
// Lidar Jacobian rows: only the first 12 error-state columns (r_wi, t_wi,
// r_il, t_il) are non-zero, so H is stored as m x 12.
using HMatX12D = Eigen::Matrix<double, Eigen::Dynamic, 12>;

M3D Jr(const V3D & inp);
M3D JrInv(const V3D & inp);

// Point-LIO output-model state: on top of the fastlio2 IESKF state, the body
// angular velocity (omg) and specific force (acc) are random-walk states so
// the IMU can be fused as a measurement instead of an input.
// Error state layout:
//   0: r_wi  3: t_wi  6: r_il  9: t_il  12: v  15: bg  18: ba  21: g
//  24: omg  27: acc
struct State
{
  static double gravity;
  M3D r_wi = M3D::Identity();
  V3D t_wi = V3D::Zero();
  M3D r_il = M3D::Identity();
  V3D t_il = V3D::Zero();
  V3D v = V3D::Zero();
  V3D bg = V3D::Zero();
  V3D ba = V3D::Zero();
  V3D g = V3D(0.0, 0.0, -9.81);
  V3D omg = V3D::Zero();
  V3D acc = V3D::Zero();

  void initGravityDir(const V3D & gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

  void operator+=(const V30D & delta);

  friend std::ostream & operator<<(std::ostream & os, const State & state);
};

class PointEKF
{
public:
  PointEKF() = default;

  void setConfig(const Config & config);

  // Nominal-state propagation with the omg/acc states; called at every event
  // (each IMU sample and each point-group timestamp).
  void predictState(double dt);

  // Covariance-only propagation; called at IMU rate (prop_at_freq_of_imu).
  void predictCov(double dt);

  // Stacked point-to-plane update for one same-timestamp point group.
  // H holds the non-zero first 12 columns of the full m x 30 Jacobian and
  // z the residuals n^T p_w + d. Single iteration, Kalman-gain form.
  void updateLidar(const HMatX12D & H, const Eigen::VectorXd & z, int rows);

  // IMU-as-measurement update (output model). Saturated axes are dropped;
  // returns false if every axis is saturated and the update is skipped.
  bool updateIMU(const V3D & gyro, const V3D & acc);

  State & x() { return m_x; }

  M30D & P() { return m_P; }

private:
  void applyUpdate(const Eigen::MatrixXd & K, const Eigen::VectorXd & y,
                   const Eigen::MatrixXd & HP);

  State m_x;
  M30D m_P;
  M30D m_F;
  M30X12D m_G;
  M12D m_Q;
  double m_satu_gyro = 35.0;
  double m_satu_acc = 30.0;
  double m_imu_meas_omg_cov = 0.1;
  double m_imu_meas_acc_cov = 0.1;
  double m_lidar_meas_cov = 0.01;
};
