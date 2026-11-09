// The filter's gravity and its accelerometer saturation (2026-11-09).
//  - g is a state with no process noise. The port started it with a 1e-4
//    covariance, so the updates could move it; at rest |g| and the
//    accelerometer bias along gravity explain the same reading, and walking
//    pushed |g| down ~0.014 an hour on robot01. With gravity_init_cov 0 (the
//    default) the covariance row of g starts zero and stays zero through every
//    prediction and update, so g never moves at all: these tests compare it
//    bit for bit, not within a tolerance.
//  - satu_acc sits just under the IMU's clip (4.004 g = 39.28 m/s^2 at
//    imu_acc_scale 9.81): a 3.5 g gait impact is used, a clipped sample is
//    not. At the old 30 m/s^2 the impact was dropped too.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include "syncai_pointlio/map_builder/imu_initializer.h"
#include "syncai_pointlio/map_builder/point_ekf.h"

namespace
{

constexpr double kG = 9.81;
// The MID360 IMU's clip in m/s^2 at imu_acc_scale 9.81.
constexpr double kClip = 4.004 * kG;
constexpr double kDt = 0.005;  // 200 Hz IMU

// A filter after the static init, the robot level and still.
std::shared_ptr<PointEKF> initializedFilter(Config config)
{
  auto kf = std::make_shared<PointEKF>();
  kf->setConfig(config);
  IMUInitializer init(config, kf);
  SyncPackage package;
  for (int i = 0; i < config.imu_init_num; ++i) {
    double t = i * kDt;
    package.imus.emplace_back(V3D(0.0, 0.0, kG), V3D::Zero(), t);
  }
  EXPECT_TRUE(init.initialize(package));
  return kf;
}

// `seconds` of a gait: IMU at 200 Hz with zero-mean impacts along z (up to
// ~3.6 g, under satu_acc), an offset of `z_offset` the bias has to explain,
// and noise on every axis; at 10 Hz a lidar-like update pinning the position
// at the origin, so the filter has to find where that offset goes.
void walk(PointEKF & kf, double seconds, double z_offset)
{
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.3);
  const int steps = static_cast<int>(seconds / kDt);
  for (int i = 0; i < steps; ++i) {
    kf.predictState(kDt);
    kf.predictCov(kDt);
    const double impact = 25.0 * std::sin(2.0 * M_PI * 2.0 * i * kDt);
    const V3D acc(noise(rng), noise(rng), kG + z_offset + impact + noise(rng));
    const V3D gyro(0.1 * noise(rng), 0.1 * noise(rng), 0.1 * noise(rng));
    kf.updateIMU(gyro, acc);
    if (i % 20 == 0) {
      HMatX12D H = HMatX12D::Zero(3, 12);
      H.block<3, 3>(0, 3) = M3D::Identity();
      const Eigen::VectorXd z = kf.x().t_wi;
      kf.updateLidar(H, z, 3);
    }
  }
}

TEST(FixedGravity, DefaultConfigHoldsGravityFixed)
{
  EXPECT_EQ(Config().gravity_init_cov, 0.0);
}

TEST(FixedGravity, GravityNeverMovesThroughImuAndLidarUpdates)
{
  auto kf = initializedFilter(Config());
  const V3D g0 = kf->x().g;
  EXPECT_DOUBLE_EQ(g0.norm(), kG);

  walk(*kf, 60.0, 0.05);

  EXPECT_EQ(kf->x().g.x(), g0.x());
  EXPECT_EQ(kf->x().g.y(), g0.y());
  EXPECT_EQ(kf->x().g.z(), g0.z());
  // Why: its covariance rows and columns never leave zero.
  EXPECT_EQ(kf->P().middleRows<3>(21).cwiseAbs().maxCoeff(), 0.0);
  EXPECT_EQ(kf->P().middleCols<3>(21).cwiseAbs().maxCoeff(), 0.0);
  // The filter itself stayed sane: the position held, the offset went into the bias.
  EXPECT_LT(kf->x().t_wi.norm(), 0.1);
  EXPECT_GT(kf->x().ba.z(), 0.0);
}

TEST(FixedGravity, AnInitialCovarianceLetsTheUpdatesMoveGravity)
{
  // The port's value: the same walk moves g, which is what drifted on robot01.
  Config config;
  config.gravity_init_cov = 0.0001;
  auto kf = initializedFilter(config);
  const V3D g0 = kf->x().g;

  walk(*kf, 60.0, 0.05);

  EXPECT_GT((kf->x().g - g0).norm(), 1e-4);
}

// A filter whose IMU prediction matches `acc` on x and y, so that only the z
// axis of an updateIMU can carry a residual.
struct ImuCase
{
  std::shared_ptr<PointEKF> kf = initializedFilter(Config());
  V3D gyro() const { return kf->x().omg + kf->x().bg; }
  V3D accWithZ(double z) const
  {
    const V3D predicted = kf->x().acc + kf->x().ba;
    return V3D(predicted.x(), predicted.y(), z);
  }
};

TEST(SatuAcc, AGaitImpactUnderTheClipIsUsed)
{
  ImuCase c;
  const V3D acc_before = c.kf->x().acc;
  ASSERT_TRUE(c.kf->updateIMU(c.gyro(), c.accWithZ(3.5 * kG)));
  EXPECT_GT(c.kf->x().acc.z(), acc_before.z() + 1.0);
}

TEST(SatuAcc, AClippedSampleIsDropped)
{
  ImuCase c;
  const V3D acc_before = c.kf->x().acc;
  const V3D ba_before = c.kf->x().ba;
  // The gyro axes are still valid, so the update runs; the z axis carries nothing.
  ASSERT_TRUE(c.kf->updateIMU(c.gyro(), c.accWithZ(kClip)));
  EXPECT_EQ(c.kf->x().acc, acc_before);
  EXPECT_EQ(c.kf->x().ba, ba_before);
}

TEST(SatuAcc, ThresholdSitsBetweenTheImpactsAndTheClip)
{
  const double satu = Config().satu_acc;
  EXPECT_GT(satu, 3.9 * kG);  // keeps impacts up to 3.9 g
  EXPECT_LT(satu, kClip);     // and still drops a clipped reading
}

}  // namespace
