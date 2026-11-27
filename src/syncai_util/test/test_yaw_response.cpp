// syncai_util::YawResponse: the gait's measured yaw response, which MPPI's
// motion model predicts with. What a change must not move: an empty table is
// the identity; a command inside the dead band turns nothing; the table's
// points are returned exactly, linear between, proportional past the last;
// the sides are separate; a table that does not start at (0, 0) or rise is
// refused.

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "syncai_util/yaw_response.hpp"

using syncai_util::YawResponse;
using syncai_util::YawResponseSide;

namespace
{

// robot01's Basic gait, 2026-11-27 (syncai_controller params/models/m20.yaml)
YawResponse robot01()
{
  YawResponse r;
  r.set(
    {{0.0, 0.5, 0.6, 0.7, 1.0}, {0.0, 0.0, 0.31, 0.53, 0.94}},
    {{0.0, 0.5, 0.6, 1.0}, {0.0, 0.0, 0.5, 0.94}});
  return r;
}

}  // namespace

TEST(YawResponse, EmptyIsTheIdentity)
{
  YawResponse r;
  EXPECT_FALSE(r.active());
  EXPECT_DOUBLE_EQ(r.rate(0.3), 0.3);
  EXPECT_DOUBLE_EQ(r.rate(-0.7), -0.7);
}

TEST(YawResponse, DeadBandTurnsNothing)
{
  const auto r = robot01();
  EXPECT_TRUE(r.active());
  EXPECT_DOUBLE_EQ(r.rate(0.0), 0.0);
  EXPECT_DOUBLE_EQ(r.rate(0.2), 0.0);
  EXPECT_DOUBLE_EQ(r.rate(-0.45), 0.0);
  EXPECT_DOUBLE_EQ(r.rate(0.5), 0.0);
}

TEST(YawResponse, PointsLinearAndProportional)
{
  const auto r = robot01();
  EXPECT_DOUBLE_EQ(r.rate(0.6), 0.31);
  EXPECT_DOUBLE_EQ(r.rate(-0.6), -0.5);
  EXPECT_NEAR(r.rate(0.65), 0.42, 1e-12);
  EXPECT_NEAR(r.rate(0.55), 0.155, 1e-12);
  // Past the last point: 0.94 / 1.0 of the command
  EXPECT_NEAR(r.rate(1.5), 1.41, 1e-12);
  EXPECT_NEAR(r.rate(-2.0), -1.88, 1e-12);
  // Odd inputs pass through
  EXPECT_TRUE(std::isnan(r.rate(NAN)));
}

TEST(YawResponse, OneSideOnly)
{
  YawResponse r;
  r.set({{0.0, 0.5, 1.0}, {0.0, 0.0, 0.9}}, {});
  EXPECT_DOUBLE_EQ(r.rate(0.4), 0.0);
  EXPECT_DOUBLE_EQ(r.rate(-0.4), -0.4);
}

TEST(YawResponse, RefusesATableItCannotUse)
{
  auto bad = [](YawResponseSide s) { return YawResponse::problem(s).has_value(); };
  EXPECT_FALSE(bad({{}, {}}));
  EXPECT_FALSE(bad({{0.0, 0.5, 1.0}, {0.0, 0.0, 0.9}}));
  EXPECT_TRUE(bad({{0.0, 0.5}, {0.0}}));
  EXPECT_TRUE(bad({{0.0}, {0.0}}));
  EXPECT_TRUE(bad({{0.1, 0.5}, {0.0, 0.4}}));
  EXPECT_TRUE(bad({{0.0, 0.5}, {0.1, 0.4}}));
  EXPECT_TRUE(bad({{0.0, 0.5, 0.5}, {0.0, 0.2, 0.4}}));
  EXPECT_TRUE(bad({{0.0, 0.5, 1.0}, {0.0, 0.4, 0.3}}));
  EXPECT_TRUE(bad({{0.0, 1.0}, {0.0, 0.0}}));
  EXPECT_TRUE(bad({{0.0, NAN}, {0.0, 0.4}}));
  YawResponse r;
  EXPECT_THROW(r.set({{0.0, 0.5}, {0.2, 0.4}}, {}), std::invalid_argument);
  EXPECT_FALSE(r.active());
}
