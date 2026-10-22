// Unit tests of localizers/localizability.*: synthetic Hessians, no ROS, no
// map. The corridor case is the one the analysis exists for; the coupling case
// is why the Schur complement is used instead of the plain block.
#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>

#include "syncai_localizer/localizers/localizability.h"

namespace
{
using M6D = Eigen::Matrix<double, 6, 6>;

// Build small_gicp's body-frame H from a map-frame information matrix, i.e.
// invert the transform analyzeLocalizability applies.
M6D bodyFromMap(const M6D & L_map, const M3D & R_map_body)
{
  M6D Rb = M6D::Zero();
  Rb.block<3, 3>(0, 0) = R_map_body;
  Rb.block<3, 3>(3, 3) = R_map_body;
  return Rb.transpose() * L_map * Rb;
}

M6D isotropic(double rot, double trans)
{
  M6D L = M6D::Zero();
  L.block<3, 3>(0, 0) = M3D::Identity() * rot;
  L.block<3, 3>(3, 3) = M3D::Identity() * trans;
  return L;
}

M3D yaw(double rad)
{
  return Eigen::AngleAxisd(rad, V3D::UnitZ()).toRotationMatrix();
}

constexpr std::size_t kInliers = 500;
}  // namespace

TEST(Localizability, IsotropicIsFullEverywhere)
{
  LocalizabilityConfig cfg;
  const M6D H = bodyFromMap(isotropic(1e4, 1e4), yaw(0.7)) * kInliers;
  const Localizability loc = analyzeLocalizability(H, yaw(0.7), kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(loc.trans_level[i], Localizable::kFull);
    EXPECT_EQ(loc.rot_level[i], Localizable::kFull);
    EXPECT_NEAR(loc.trans_eig(i), 1e4, 1e-6);  // per inlier
  }
  const V3D d(0.1, -0.2, 0.05);
  EXPECT_TRUE(projectTranslation(loc, d, cfg).isApprox(d));
  EXPECT_TRUE(projectRotation(loc, d, cfg).isApprox(d));
  EXPECT_FALSE(loc.transFullyDegenerate());
  EXPECT_EQ(loc.levelString(), "FFF/FFF");
}

TEST(Localizability, CorridorAlongMapXIsNoneAlongXWhateverTheHeading)
{
  LocalizabilityConfig cfg;
  // Walls parallel to map x: strong y / z translation information, none along
  // x. The robot faces 30 deg off the corridor; the weak direction must still
  // come out as map x.
  M6D L = isotropic(1e4, 1e4);
  L(3, 3) = 1.0;  // t_x, per inlier: ratio 1e-4 << ratio_none
  const M3D R = yaw(30.0 * M_PI / 180.0);
  const Localizability loc = analyzeLocalizability(bodyFromMap(L, R) * kInliers, R, kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  EXPECT_EQ(loc.trans_level[0], Localizable::kNone);
  EXPECT_EQ(loc.trans_level[1], Localizable::kFull);
  EXPECT_EQ(loc.trans_level[2], Localizable::kFull);
  EXPECT_NEAR(std::abs(loc.trans_vec.col(0).x()), 1.0, 1e-6);
  EXPECT_NEAR(loc.weakestTransHeading(), 0.0, 1e-6);

  // The correction along the corridor is dropped, the rest kept.
  const V3D d(0.3, 0.02, -0.01);
  const V3D p = projectTranslation(loc, d, cfg);
  EXPECT_NEAR(p.x(), 0.0, 1e-9);
  EXPECT_NEAR(p.y(), d.y(), 1e-9);
  EXPECT_NEAR(p.z(), d.z(), 1e-9);
  EXPECT_FALSE(loc.transFullyDegenerate());
  EXPECT_EQ(loc.levelString(), "NFF/FFF");
}

TEST(Localizability, PartialDirectionIsDamped)
{
  LocalizabilityConfig cfg;
  cfg.partial_gain = 0.25;
  M6D L = isotropic(1e4, 1e4);
  L(4, 4) = 1e4 * 0.05;  // t_y: ratio 0.05, between ratio_none and ratio_partial
  const Localizability loc = analyzeLocalizability(
    bodyFromMap(L, M3D::Identity()) * kInliers, M3D::Identity(), kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  EXPECT_EQ(loc.trans_level[0], Localizable::kPartial);
  const V3D p = projectTranslation(loc, V3D(1.0, 1.0, 1.0), cfg);
  EXPECT_NEAR(p.x(), 1.0, 1e-9);
  EXPECT_NEAR(p.y(), 0.25, 1e-9);
  EXPECT_NEAR(p.z(), 1.0, 1e-9);
}

TEST(Localizability, YawCouplingWeakensLateralTranslation)
{
  // Plain blocks say t_y is as strong as t_x; once yaw can absorb it (a wall
  // far away, seen at an angle), the marginal says otherwise.
  LocalizabilityConfig cfg;
  M6D L = isotropic(100.0, 100.0);
  L(2, 4) = L(4, 2) = 99.0;  // r_z <-> t_y
  const Localizability loc = analyzeLocalizability(
    bodyFromMap(L, M3D::Identity()) * kInliers, M3D::Identity(), kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  EXPECT_NEAR(loc.trans_eig(0), 100.0 - 99.0 * 99.0 / 100.0, 1e-6);  // 1.99
  EXPECT_NEAR(std::abs(loc.trans_vec.col(0).y()), 1.0, 1e-6);
  EXPECT_EQ(loc.trans_level[0], Localizable::kNone);  // 1.99 / 100 < ratio_none
  EXPECT_NEAR(loc.rot_eig(0), 1.99, 1e-6);
  EXPECT_EQ(loc.rot_level[0], Localizable::kNone);
}

TEST(Localizability, RotationProjectionUsesTheRotationBlock)
{
  LocalizabilityConfig cfg;
  M6D L = isotropic(1e4, 1e4);
  L(2, 2) = 1.0;  // yaw unconstrained (a round room)
  const Localizability loc =
    analyzeLocalizability(bodyFromMap(L, yaw(1.2)) * kInliers, yaw(1.2), kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  EXPECT_EQ(loc.rot_level[0], Localizable::kNone);
  EXPECT_NEAR(std::abs(loc.rot_vec.col(0).z()), 1.0, 1e-6);
  const V3D p = projectRotation(loc, V3D(0.01, 0.02, 0.5), cfg);
  EXPECT_NEAR(p.z(), 0.0, 1e-9);
  EXPECT_NEAR(p.x(), 0.01, 1e-9);
  // Translation untouched by a rotation-only degeneracy.
  const V3D d(0.1, 0.2, 0.3);
  EXPECT_TRUE(projectTranslation(loc, d, cfg).isApprox(d));
}

TEST(Localizability, AllWeakIsFullyDegenerate)
{
  LocalizabilityConfig cfg;
  cfg.min_eig_per_inlier = 10.0;
  const M6D L = isotropic(1e4, 1.0);  // every translation direction below the floor
  const Localizability loc = analyzeLocalizability(
    bodyFromMap(L, M3D::Identity()) * kInliers, M3D::Identity(), kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  EXPECT_TRUE(loc.transFullyDegenerate());
  EXPECT_TRUE(projectTranslation(loc, V3D(1, 1, 1), cfg).isZero());
}

TEST(Localizability, InvalidInputIsIdentity)
{
  LocalizabilityConfig cfg;
  const V3D d(0.1, 0.2, 0.3);
  const Localizability none = analyzeLocalizability(M6D::Zero(), M3D::Identity(), kInliers, cfg);
  EXPECT_FALSE(none.valid);
  EXPECT_TRUE(none.transFullyDegenerate());
  EXPECT_TRUE(projectTranslation(none, d, cfg).isApprox(d));
  const Localizability no_inliers = analyzeLocalizability(isotropic(1, 1), M3D::Identity(), 0, cfg);
  EXPECT_FALSE(no_inliers.valid);
  EXPECT_TRUE(projectRotation(no_inliers, d, cfg).isApprox(d));
}

TEST(Localizability, DisabledKeepsTheAnalysisButNotTheProjection)
{
  LocalizabilityConfig cfg;
  cfg.enabled = false;
  M6D L = isotropic(1e4, 1e4);
  L(3, 3) = 1.0;
  const Localizability loc = analyzeLocalizability(
    bodyFromMap(L, M3D::Identity()) * kInliers, M3D::Identity(), kInliers, cfg);
  ASSERT_TRUE(loc.valid);
  EXPECT_EQ(loc.trans_level[0], Localizable::kNone);  // still reported, for the log
  const V3D d(0.3, 0.02, -0.01);
  EXPECT_TRUE(projectTranslation(loc, d, cfg).isApprox(d));  // but not applied
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
