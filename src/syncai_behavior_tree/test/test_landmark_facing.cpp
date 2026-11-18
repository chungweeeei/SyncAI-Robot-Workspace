// faceLandmarkToSensor(): the pose a sighting with an untrusted orientation
// is published as. ROS-free.

#include <cmath>
#include <random>

#include "gtest/gtest.h"
#include "syncai_behavior_tree/landmark_facing.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace
{

using syncai_behavior_tree::faceLandmarkToSensor;
using syncai_behavior_tree::landmarkIsNear;
using syncai_behavior_tree::landmarkRange;

// A camera optical frame: z ahead, x right, y down.
const tf2::Vector3 kOpticalUp(0.0, -1.0, 0.0);

// A landmark at `position` whose +x is `normal` and whose +z is as close to `up` as that allows.
geometry_msgs::msg::Pose makePose(
  const tf2::Vector3 & position, tf2::Vector3 normal, const tf2::Vector3 & up)
{
  normal.normalize();
  tf2::Vector3 z = up - normal * up.dot(normal);
  z.normalize();
  const tf2::Vector3 y = z.cross(normal);
  tf2::Quaternion q;
  tf2::Matrix3x3(
    normal.x(), y.x(), z.x(), normal.y(), y.y(), z.y(), normal.z(), y.z(), z.z()).getRotation(q);
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

tf2::Vector3 axisOf(const geometry_msgs::msg::Pose & pose, int column)
{
  const tf2::Quaternion q(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  return tf2::Matrix3x3(q).getColumn(column);
}

// The normal reflected about the viewing ray: the other PnP solution of a planar marker.
tf2::Vector3 mirrorAboutRay(const tf2::Vector3 & normal, const tf2::Vector3 & position)
{
  const tf2::Vector3 ray = (-position).normalized();
  return ray * (2.0 * normal.dot(ray)) - normal;
}

TEST(LandmarkFacing, StraightAheadFacesTheSensorAndKeepsUp)
{
  const tf2::Vector3 position(0.0, 0.0, 1.2);
  // Detected 10 deg off the ray, about the vertical.
  const tf2::Vector3 normal(std::sin(0.175), 0.0, -std::cos(0.175));
  const auto facing = faceLandmarkToSensor(makePose(position, normal, kOpticalUp));
  ASSERT_TRUE(facing.has_value());
  EXPECT_NEAR(axisOf(*facing, 0).dot(tf2::Vector3(0.0, 0.0, -1.0)), 1.0, 1e-9);
  EXPECT_NEAR(axisOf(*facing, 2).dot(kOpticalUp), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(facing->position.z, 1.2);
}

TEST(LandmarkFacing, OffToEitherSideTheNormalPointsAtTheSensor)
{
  for (const double x : {-0.3, 0.3}) {
    const tf2::Vector3 position(x, 0.05, 1.4);
    const auto facing =
      faceLandmarkToSensor(makePose(position, tf2::Vector3(0.1, 0.0, -1.0), kOpticalUp));
    ASSERT_TRUE(facing.has_value());
    EXPECT_NEAR(axisOf(*facing, 0).dot((-position).normalized()), 1.0, 1e-9);
    EXPECT_DOUBLE_EQ(facing->position.x, x);
  }
}

TEST(LandmarkFacing, BothMirroredSolutionsGiveOnePose)
{
  // robot01, 2026-11-18: the tag 1.2 m ahead, its normal ~7 deg off the ray.
  const tf2::Vector3 position(0.02, 0.04, 1.18);
  const tf2::Vector3 normal(std::sin(0.12), 0.0, -std::cos(0.12));
  const tf2::Vector3 mirrored = mirrorAboutRay(normal, position);
  const auto a = faceLandmarkToSensor(makePose(position, normal, kOpticalUp));
  const auto b = faceLandmarkToSensor(makePose(position, mirrored, kOpticalUp));
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_NEAR(axisOf(*a, 0).dot(axisOf(*b, 0)), 1.0, 1e-9);
  // The detected normals themselves are 2 x the angle off the ray apart.
  EXPECT_LT(normal.dot(mirrored), std::cos(0.2));
}

TEST(LandmarkFacing, APitchedSensorStillGetsAnOrthonormalFrame)
{
  // The landmark below the optical axis, as from a camera pitched up.
  const tf2::Vector3 position(0.1, 0.4, 1.0);
  const auto facing =
    faceLandmarkToSensor(makePose(position, tf2::Vector3(0.0, -0.2, -1.0), kOpticalUp));
  ASSERT_TRUE(facing.has_value());
  const tf2::Vector3 x = axisOf(*facing, 0);
  const tf2::Vector3 z = axisOf(*facing, 2);
  EXPECT_NEAR(x.dot(z), 0.0, 1e-9);
  EXPECT_NEAR(x.dot((-position).normalized()), 1.0, 1e-9);
  EXPECT_GT(z.dot(kOpticalUp), 0.9);
}

TEST(LandmarkFacing, NoPoseWhereNoneExists)
{
  // At the sensor's origin.
  EXPECT_FALSE(faceLandmarkToSensor(
      makePose(tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(0.0, 0.0, -1.0), kOpticalUp)));
  // The detected up along the ray.
  EXPECT_FALSE(faceLandmarkToSensor(
      makePose(tf2::Vector3(0.0, -1.0, 0.0), tf2::Vector3(0.0, 0.0, -1.0), kOpticalUp)));
  // A zero quaternion.
  geometry_msgs::msg::Pose zero;
  zero.position.z = 1.0;
  zero.orientation.w = 0.0;
  EXPECT_FALSE(faceLandmarkToSensor(zero));
}

TEST(LandmarkFacing, StressRandomSightings)
{
  std::mt19937 generator(20261118);
  std::uniform_real_distribution<double> lateral(-1.0, 1.0);
  std::uniform_real_distribution<double> depth(0.2, 4.0);
  std::uniform_real_distribution<double> tilt(-0.6, 0.6);
  for (int i = 0; i < 100000; ++i) {
    const tf2::Vector3 position(lateral(generator), lateral(generator), depth(generator));
    const tf2::Vector3 normal(tilt(generator), tilt(generator), -1.0);
    const geometry_msgs::msg::Pose detected = makePose(position, normal, kOpticalUp);
    const auto facing = faceLandmarkToSensor(detected);
    ASSERT_TRUE(facing.has_value());
    const tf2::Quaternion q(
      facing->orientation.x, facing->orientation.y, facing->orientation.z,
      facing->orientation.w);
    ASSERT_NEAR(q.length(), 1.0, 1e-9);
    ASSERT_NEAR(axisOf(*facing, 0).dot((-position).normalized()), 1.0, 1e-9);
    ASSERT_GT(axisOf(*facing, 2).dot(axisOf(detected, 2)), 0.0);
  }
}

TEST(LandmarkNear, RangeIsTheLengthOfThePosition)
{
  const auto pose =
    makePose(tf2::Vector3(0.3, -0.4, 1.2), tf2::Vector3(0.0, 0.0, -1.0), kOpticalUp);
  EXPECT_NEAR(landmarkRange(pose), 1.3, 1e-12);
}

TEST(LandmarkNear, EntersBelowTheRangeAndLeavesOnlyBeyondTheHysteresis)
{
  // Walking in: far until under 1.0 m.
  EXPECT_FALSE(landmarkIsNear(false, 1.05, 1.0, 0.1));
  EXPECT_FALSE(landmarkIsNear(false, 1.0, 1.0, 0.1));
  EXPECT_TRUE(landmarkIsNear(false, 0.99, 1.0, 0.1));
  // Stepping back a little: still near up to 1.1 m.
  EXPECT_TRUE(landmarkIsNear(true, 1.08, 1.0, 0.1));
  EXPECT_TRUE(landmarkIsNear(true, 1.1, 1.0, 0.1));
  EXPECT_FALSE(landmarkIsNear(true, 1.11, 1.0, 0.1));
}

TEST(LandmarkNear, ZeroRangeIsNeverNearAndANegativeHysteresisIsNone)
{
  EXPECT_FALSE(landmarkIsNear(false, 0.1, 0.0, 0.1));
  EXPECT_FALSE(landmarkIsNear(true, 0.1, 0.0, 0.1));
  EXPECT_FALSE(landmarkIsNear(true, 1.01, 1.0, -0.5));
  EXPECT_TRUE(landmarkIsNear(true, 1.0, 1.0, -0.5));
}

TEST(LandmarkNear, StressASwayInsideTheHysteresisNeverChangesTheAnswer)
{
  std::mt19937 generator(20261118);
  std::uniform_real_distribution<double> sway(-0.04, 0.04);
  for (int run = 0; run < 1000; ++run) {
    // Standing 2 cm inside the line, swaying 4 cm either way.
    bool near = landmarkIsNear(false, 0.98, 1.0, 0.1);
    ASSERT_TRUE(near);
    for (int i = 0; i < 300; ++i) {
      near = landmarkIsNear(near, 0.98 + sway(generator), 1.0, 0.1);
      ASSERT_TRUE(near);
    }
  }
}

}  // namespace
