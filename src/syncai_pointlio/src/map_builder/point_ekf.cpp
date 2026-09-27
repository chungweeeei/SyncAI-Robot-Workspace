#include "syncai_pointlio/map_builder/point_ekf.h"

double State::gravity = 9.81;

M3D Jr(const V3D & inp)
{
  return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D & inp)
{
  return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V30D & delta)
{
  r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
  t_wi += delta.segment<3>(3);
  r_il *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
  t_il += delta.segment<3>(9);
  v += delta.segment<3>(12);
  bg += delta.segment<3>(15);
  ba += delta.segment<3>(18);
  g += delta.segment<3>(21);
  omg += delta.segment<3>(24);
  acc += delta.segment<3>(27);
}

std::ostream & operator<<(std::ostream & os, const State & state)
{
  os << "==============START===============" << std::endl;
  os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
  os << "t_wi: " << state.t_wi.transpose() << std::endl;
  os << "r_il: " << state.r_il.eulerAngles(2, 1, 0).transpose() << std::endl;
  os << "t_il: " << state.t_il.transpose() << std::endl;
  os << "v: " << state.v.transpose() << std::endl;
  os << "bg: " << state.bg.transpose() << std::endl;
  os << "ba: " << state.ba.transpose() << std::endl;
  os << "g: " << state.g.transpose() << std::endl;
  os << "omg: " << state.omg.transpose() << std::endl;
  os << "acc: " << state.acc.transpose() << std::endl;
  os << "===============END================" << std::endl;

  return os;
}

void PointEKF::setConfig(const Config & config)
{
  m_Q.setZero();
  m_Q.block<3, 3>(0, 0) = M3D::Identity() * config.gyr_cov_output;
  m_Q.block<3, 3>(3, 3) = M3D::Identity() * config.acc_cov_output;
  m_Q.block<3, 3>(6, 6) = M3D::Identity() * config.b_gyr_cov;
  m_Q.block<3, 3>(9, 9) = M3D::Identity() * config.b_acc_cov;
  m_satu_gyro = config.satu_gyro;
  m_satu_acc = config.satu_acc;
  m_imu_meas_omg_cov = config.imu_meas_omg_cov;
  m_imu_meas_acc_cov = config.imu_meas_acc_cov;
  m_lidar_meas_cov = config.lidar_meas_cov;
  m_P.setIdentity();
}

void PointEKF::predictState(double dt)
{
  if (dt <= 0.0) return;
  V30D delta = V30D::Zero();
  delta.segment<3>(0) = m_x.omg * dt;
  delta.segment<3>(3) = m_x.v * dt;
  delta.segment<3>(12) = (m_x.r_wi * m_x.acc + m_x.g) * dt;
  m_x += delta;
}

void PointEKF::predictCov(double dt)
{
  if (dt <= 0.0) return;
  m_F.setIdentity();
  m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-m_x.omg * dt).matrix();
  m_F.block<3, 3>(0, 24) = Jr(m_x.omg * dt) * dt;
  m_F.block<3, 3>(3, 12) = M3D::Identity() * dt;
  m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(m_x.acc) * dt;
  m_F.block<3, 3>(12, 21) = M3D::Identity() * dt;
  m_F.block<3, 3>(12, 27) = m_x.r_wi * dt;

  m_G.setZero();
  m_G.block<3, 3>(24, 0) = M3D::Identity() * dt;
  m_G.block<3, 3>(27, 3) = M3D::Identity() * dt;
  m_G.block<3, 3>(15, 6) = M3D::Identity() * dt;
  m_G.block<3, 3>(18, 9) = M3D::Identity() * dt;

  m_P = m_F * m_P * m_F.transpose() + m_G * m_Q * m_G.transpose();
}

void PointEKF::updateLidar(const HMatX12D & H, const Eigen::VectorXd & z, int rows)
{
  if (rows < 1) return;
  // Full Jacobian is [H 0] (m x 30): H P = H * P(0:12, :)
  Eigen::MatrixXd HP = H.topRows(rows) * m_P.topRows<12>();
  Eigen::MatrixXd S = HP.leftCols<12>() * H.topRows(rows).transpose();
  S.diagonal().array() += m_lidar_meas_cov;
  // K = P H^T S^-1 = (S^-1 H P)^T since P and S are symmetric
  Eigen::MatrixXd K = S.ldlt().solve(HP).transpose();
  applyUpdate(K, -z.head(rows), HP);
}

bool PointEKF::updateIMU(const V3D & gyro, const V3D & acc)
{
  Eigen::Matrix<double, 6, 1> y;
  y.head<3>() = gyro - (m_x.omg + m_x.bg);
  y.tail<3>() = acc - (m_x.acc + m_x.ba);

  Eigen::Matrix<double, 6, 30> H;
  H.setZero();
  Eigen::Matrix<double, 6, 1> r_diag;
  int valid_num = 0;
  for (int i = 0; i < 3; i++) {
    r_diag(i) = m_imu_meas_omg_cov;
    r_diag(3 + i) = m_imu_meas_acc_cov;
    if (std::fabs(gyro(i)) < m_satu_gyro) {
      H(i, 15 + i) = 1.0;
      H(i, 24 + i) = 1.0;
      valid_num++;
    } else {
      y(i) = 0.0;
    }
    if (std::fabs(acc(i)) < m_satu_acc) {
      H(3 + i, 18 + i) = 1.0;
      H(3 + i, 27 + i) = 1.0;
      valid_num++;
    } else {
      y(3 + i) = 0.0;
    }
  }
  if (valid_num == 0) return false;

  Eigen::Matrix<double, 6, 30> HP = H * m_P;
  Eigen::Matrix<double, 6, 6> S = HP * H.transpose();
  S.diagonal() += r_diag;
  Eigen::Matrix<double, 30, 6> K = S.ldlt().solve(HP).transpose();
  applyUpdate(K, y, HP);
  return true;
}

void PointEKF::applyUpdate(const Eigen::MatrixXd & K, const Eigen::VectorXd & y,
                           const Eigen::MatrixXd & HP)
{
  V30D delta = K * y;
  m_x += delta;
  m_P -= K * HP;

  M30D L = M30D::Identity();
  L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
  L.block<3, 3>(6, 6) = Jr(delta.segment<3>(6));
  m_P = L * m_P * L.transpose();
  m_P = 0.5 * (m_P + m_P.transpose());
}
