// mppi::utils::findClosestPathPt, the integrated-distance lookup
// PathAlignCritic walks each trajectory along. Humble's version read past the
// vector's end once the trajectory outran the path, and answered 0 instead of
// the start index; see its comment in utils.hpp.

#include <gtest/gtest.h>

#include <vector>

#include "syncai_mppi_controller/tools/utils.hpp"

using mppi::utils::findClosestPathPt;

namespace
{
// A path of 0.1 m segments: integrated distances 0, 0.1, ..., 0.5.
const std::vector<float> kPath{0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f};
}  // namespace

TEST(FindClosestPathPt, PicksTheNearerOfTheTwoNeighbours)
{
  EXPECT_EQ(findClosestPathPt(kPath, 0.0f), 0u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.04f), 0u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.06f), 1u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.21f), 2u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.29f), 3u);
}

TEST(FindClosestPathPt, PastTheEndClampsToTheLastPoint)
{
  // Humble's lower_bound returned end() here and dereferenced it.
  EXPECT_EQ(findClosestPathPt(kPath, 0.5f), 5u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.7f), 5u);
  EXPECT_EQ(findClosestPathPt(kPath, 10.0f, 3), 5u);
  EXPECT_EQ(findClosestPathPt(kPath, 10.0f, 5), 5u);
}

TEST(FindClosestPathPt, NeverAnswersBeforeTheStartIndex)
{
  // Humble's answered 0 whenever the closest point was init itself, which
  // sent the rest of the trajectory back to the path's start.
  EXPECT_EQ(findClosestPathPt(kPath, 0.31f, 3), 3u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.0f, 3), 3u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.36f, 3), 4u);
}

TEST(FindClosestPathPt, WalkingATrajectoryOnlyMovesForward)
{
  // PathAlignCritic feeds each answer back as the next init.
  size_t pt = 0;
  size_t last = 0;
  for (float d = 0.0f; d < 1.0f; d += 0.03f) {
    pt = findClosestPathPt(kPath, d, pt);
    EXPECT_GE(pt, last);
    EXPECT_LT(pt, kPath.size());
    last = pt;
  }
  EXPECT_EQ(pt, 5u);
}

TEST(FindClosestPathPt, DegenerateVectors)
{
  EXPECT_EQ(findClosestPathPt({}, 1.0f), 0u);
  EXPECT_EQ(findClosestPathPt({0.0f}, 1.0f), 0u);
  EXPECT_EQ(findClosestPathPt(kPath, 0.2f, 9), 5u);
}
