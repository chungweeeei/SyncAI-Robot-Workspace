// IVoxMap against a brute-force oracle. The map is what every scan point is
// matched against, so what its search returns and what it keeps are the two
// things a change to it must not move by accident: the k nearest inside one
// grid and across its face neighbours (exact there), the corner and edge
// grids it deliberately does not look in (approximate by design, as
// upstream), and the least-recently-inserted-into eviction order.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "syncai_pointlio/map_builder/ivox_map.h"

namespace
{

PointType makePoint(float x, float y, float z)
{
  PointType p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 0.0f;
  return p;
}

int gridIndex(float v, double resolution)
{
  return static_cast<int>(std::floor(v / resolution));
}

// The grids a search looks in: the query's own and its 6 face neighbours.
bool inFaceNeighbourhood(const PointType & q, const PointType & p, double resolution)
{
  const int dx = std::abs(gridIndex(p.x, resolution) - gridIndex(q.x, resolution));
  const int dy = std::abs(gridIndex(p.y, resolution) - gridIndex(q.y, resolution));
  const int dz = std::abs(gridIndex(p.z, resolution) - gridIndex(q.z, resolution));
  return dx + dy + dz <= 1;
}

// Squared distances of the k nearest candidates, ascending: what nearestSearch
// must return, computed the slow way.
std::vector<float> oracleDistances(
  const PointType & q, const PointVec & points, int k, double max_range, double resolution)
{
  std::vector<float> d;
  const float max_sq = static_cast<float>(max_range * max_range);
  for (const PointType & p : points) {
    if (!inFaceNeighbourhood(q, p, resolution)) continue;
    const float s = sq_dist(p, q);
    if (s < max_sq) d.push_back(s);
  }
  std::sort(d.begin(), d.end());
  if (static_cast<int>(d.size()) > k) d.resize(k);
  return d;
}

PointVec randomPoints(std::mt19937 & rng, int n, float lo, float hi)
{
  std::uniform_real_distribution<float> u(lo, hi);
  PointVec points;
  points.reserve(n);
  for (int i = 0; i < n; ++i) points.push_back(makePoint(u(rng), u(rng), u(rng)));
  return points;
}

}  // namespace

// Inside one grid the face-neighbour search is exact: it must agree with the
// oracle point for point, sorted ascending, with distances that are the
// points' own.
TEST(IVoxMap, NearestInsideOneGridMatchesBruteForce)
{
  const double resolution = 2.0;
  IVoxMap map(resolution, 1000, 6);
  std::mt19937 rng(7);
  PointVec points = randomPoints(rng, 400, 0.3f, 1.7f);
  map.addPoints(points);
  EXPECT_EQ(map.numGrids(), 1u);
  EXPECT_EQ(map.numPoints(), 400u);

  PointVec nearest;
  std::vector<float> distances;
  for (int i = 0; i < 50; ++i) {
    const PointType q = randomPoints(rng, 1, 0.3f, 1.7f)[0];
    map.nearestSearch(q, 5, 5.0, nearest, distances);
    const std::vector<float> expected = oracleDistances(q, points, 5, 5.0, resolution);
    ASSERT_EQ(distances.size(), expected.size());
    ASSERT_EQ(nearest.size(), distances.size());
    for (size_t j = 0; j < expected.size(); ++j) {
      EXPECT_FLOAT_EQ(distances[j], expected[j]);
      EXPECT_FLOAT_EQ(sq_dist(nearest[j], q), distances[j]);
      if (j > 0) {
        EXPECT_LE(distances[j - 1], distances[j]);
      }
    }
  }
}

// A point in a face-neighbour grid is found; a nearer one in a corner grid is
// not. That is the approximation upstream accepts for the speed, and a search
// that silently widened would cost every query more.
TEST(IVoxMap, SearchesFaceNeighboursAndNotCorners)
{
  IVoxMap map(2.0, 1000, 6);
  const PointType query = makePoint(1.9f, 1.9f, 1.0f);   // grid (0, 0, 0)
  const PointType face = makePoint(2.4f, 1.9f, 1.0f);    // grid (1, 0, 0), 0.5 m away
  const PointType corner = makePoint(2.1f, 2.1f, 1.0f);  // grid (1, 1, 0), 0.28 m away
  PointVec points = {face, corner};
  map.addPoints(points);

  PointVec nearest;
  std::vector<float> distances;
  map.nearestSearch(query, 5, 5.0, nearest, distances);
  ASSERT_EQ(nearest.size(), 1u);
  EXPECT_FLOAT_EQ(nearest[0].x, face.x);
  EXPECT_FLOAT_EQ(distances[0], sq_dist(face, query));
}

// max_range is a strict bound and k a cap: nothing at or beyond the range,
// no more than k, all of them when there are fewer.
TEST(IVoxMap, RangeAndCountBounds)
{
  IVoxMap map(2.0, 1000, 6);
  const PointType query = makePoint(1.0f, 1.0f, 1.0f);
  PointVec points = {
    makePoint(1.1f, 1.0f, 1.0f), makePoint(1.3f, 1.0f, 1.0f), makePoint(1.0f, 1.6f, 1.0f),
    makePoint(1.0f, 1.0f, 1.9f)};
  map.addPoints(points);

  PointVec nearest;
  std::vector<float> distances;
  map.nearestSearch(query, 2, 5.0, nearest, distances);
  EXPECT_EQ(nearest.size(), 2u);
  EXPECT_FLOAT_EQ(distances[0], 0.1f * 0.1f);
  EXPECT_FLOAT_EQ(distances[1], 0.3f * 0.3f);

  map.nearestSearch(query, 10, 0.5, nearest, distances);
  EXPECT_EQ(nearest.size(), 2u);

  map.nearestSearch(query, 10, 5.0, nearest, distances);
  EXPECT_EQ(nearest.size(), 4u);

  map.nearestSearch(query, 0, 5.0, nearest, distances);
  EXPECT_TRUE(nearest.empty());
}

// Past capacity the grid longest without an insertion goes, whole; a grid
// inserted into again moves to the front and survives a younger one.
TEST(IVoxMap, EvictsTheLeastRecentlyInsertedGrid)
{
  IVoxMap map(1.0, 2, 6);
  PointVec a = {makePoint(0.5f, 0.5f, 0.5f), makePoint(0.6f, 0.5f, 0.5f)};
  PointVec b = {makePoint(1.5f, 0.5f, 0.5f)};
  PointVec c = {makePoint(2.5f, 0.5f, 0.5f)};
  PointVec d = {makePoint(3.5f, 0.5f, 0.5f)};

  map.addPoints(a);
  map.addPoints(b);
  EXPECT_EQ(map.numGrids(), 2u);
  EXPECT_EQ(map.numPoints(), 3u);
  EXPECT_EQ(map.numEvicted(), 0u);

  map.addPoints(c);  // a is the oldest and goes with both its points
  EXPECT_EQ(map.numGrids(), 2u);
  EXPECT_EQ(map.numPoints(), 2u);
  EXPECT_EQ(map.numEvicted(), 1u);

  PointVec nearest;
  std::vector<float> distances;
  map.nearestSearch(makePoint(0.5f, 0.5f, 0.5f), 5, 5.0, nearest, distances);
  EXPECT_FALSE(nearest.empty());
  EXPECT_FLOAT_EQ(nearest[0].x, 1.5f);  // b's point, not a's

  PointVec b_again = {makePoint(1.6f, 0.5f, 0.5f)};
  map.addPoints(b_again);  // b to the front
  map.addPoints(d);        // so c, not b, is the one dropped
  EXPECT_EQ(map.numGrids(), 2u);
  EXPECT_EQ(map.numEvicted(), 2u);
  map.nearestSearch(makePoint(2.5f, 0.5f, 0.5f), 5, 5.0, nearest, distances);
  for (const PointType & p : nearest) EXPECT_NE(p.x, 2.5f);
}

// Stress: many grids, many queries, the oracle restricted to the same seven
// grids. Ties aside (a random cloud has none), the distance lists must match.
TEST(IVoxMap, StressAgainstOracleAcrossGrids)
{
  const double resolution = 2.0;
  IVoxMap map(resolution, 1000000, 6);
  std::mt19937 rng(11);
  PointVec points = randomPoints(rng, 60000, -4.0f, 4.0f);
  map.addPoints(points);
  EXPECT_EQ(map.numPoints(), points.size());
  EXPECT_EQ(map.numGrids(), 64u);

  PointVec nearest;
  std::vector<float> distances;
  for (int i = 0; i < 300; ++i) {
    const PointType q = randomPoints(rng, 1, -4.0f, 4.0f)[0];
    map.nearestSearch(q, 5, 5.0, nearest, distances);
    const std::vector<float> expected = oracleDistances(q, points, 5, 5.0, resolution);
    ASSERT_EQ(distances.size(), expected.size());
    for (size_t j = 0; j < expected.size(); ++j) {
      EXPECT_FLOAT_EQ(distances[j], expected[j]);
      EXPECT_FLOAT_EQ(sq_dist(nearest[j], q), distances[j]);
    }
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
