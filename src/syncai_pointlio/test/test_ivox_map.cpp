// IVoxMap against a brute-force oracle. The map is what every scan point is
// matched against, so what its search returns and what it keeps are the
// things a change to it must not move by accident:
//  - the k nearest inside one grid and across its face neighbours (exact
//    there), the corner and edge grids it deliberately does not look in
//    (approximate by design, as upstream), and the least-recently-inserted-
//    into eviction order;
//  - one point per voxel, held by the map itself: the first point in a voxel
//    stays and every later one is refused, across grid boundaries and inside
//    one batch, so a wall seen a thousand times holds as many points as it
//    has voxels. The rule used to be judged outside the map from the update's
//    5 nearest neighbours, which let a voxel take a second point whenever its
//    first was not among them; on robot01 the map reached 290 points per 2 m
//    grid in 70 h and the search slowed until scans were skipped.
// The search tests use a 1 mm voxel, fine enough that no two points of their
// random clouds share one, so they see every point they insert.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <tuple>
#include <vector>

#include "syncai_pointlio/map_builder/ivox_map.h"

namespace
{

constexpr double kGrid = 2.0;
constexpr double kVoxel = 0.3;
constexpr double kFineVoxel = 0.001;

PointType makePoint(float x, float y, float z)
{
  PointType p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 0.0f;
  return p;
}

int cellIndex(float v, double resolution)
{
  return static_cast<int>(std::floor(v * (1.0 / resolution)));
}

// The grids a search looks in: the query's own and its 6 face neighbours.
bool inFaceNeighbourhood(const PointType & q, const PointType & p, double resolution)
{
  const int dx = std::abs(cellIndex(p.x, resolution) - cellIndex(q.x, resolution));
  const int dy = std::abs(cellIndex(p.y, resolution) - cellIndex(q.y, resolution));
  const int dz = std::abs(cellIndex(p.z, resolution) - cellIndex(q.z, resolution));
  return dx + dy + dz <= 1;
}

// Squared distances of the k nearest candidates, ascending: what nearestSearch
// must return, computed the slow way.
std::vector<float> oracleDistances(
  const PointType & q, const PointVec & points, int k, double max_range, double resolution)
{
  std::vector<float> distances;
  const float max_sq = static_cast<float>(max_range * max_range);
  for (const PointType & p : points) {
    if (!inFaceNeighbourhood(q, p, resolution)) continue;
    const float sq = sq_dist(p, q);
    if (sq < max_sq) distances.push_back(sq);
  }
  std::sort(distances.begin(), distances.end());
  if (static_cast<int>(distances.size()) > k) distances.resize(k);
  return distances;
}

PointVec randomPoints(std::mt19937 & rng, int n, float lo, float hi)
{
  std::uniform_real_distribution<float> uniform(lo, hi);
  PointVec points;
  points.reserve(n);
  for (int i = 0; i < n; ++i) points.push_back(makePoint(uniform(rng), uniform(rng), uniform(rng)));
  return points;
}

using VoxelIndex = std::tuple<int, int, int>;

VoxelIndex voxelIndex(const PointType & p, double resolution)
{
  return {cellIndex(p.x, resolution), cellIndex(p.y, resolution), cellIndex(p.z, resolution)};
}

}  // namespace

// Inside one grid the face-neighbour search is exact: it must agree with the
// oracle point for point, sorted ascending, with distances that are the
// points' own.
TEST(IVoxMap, NearestInsideOneGridMatchesBruteForce)
{
  IVoxMap map(kGrid, kFineVoxel, 1000, 6);
  std::mt19937 rng(7);
  PointVec points = randomPoints(rng, 400, 0.3f, 1.7f);
  map.addPoints(points);
  EXPECT_EQ(map.numGrids(), 1u);
  EXPECT_EQ(map.numPoints(), 400u);
  EXPECT_EQ(map.numRefused(), 0u);

  PointVec nearest;
  std::vector<float> distances;
  for (int i = 0; i < 50; ++i) {
    const PointType q = randomPoints(rng, 1, 0.3f, 1.7f)[0];
    map.nearestSearch(q, 5, 5.0, nearest, distances);
    const std::vector<float> expected = oracleDistances(q, points, 5, 5.0, kGrid);
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
  IVoxMap map(kGrid, kFineVoxel, 1000, 6);
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
  IVoxMap map(kGrid, kFineVoxel, 1000, 6);
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

// The update keeps a point only when its k-th neighbour is within sqrt(5) m,
// so a search bounded at sqrt(5) m must select exactly what a 5 m search
// followed by that gate selects: the same k points when the gate passes,
// fewer than k (a rejection) when it does not.
TEST(IVoxMap, SqrtFiveRangeSelectsWhatTheGateKeeps)
{
  IVoxMap map(kGrid, kFineVoxel, 1000000, 6);
  std::mt19937 rng(5);
  // Sparse enough that about a third of the queries fail the gate.
  map.addPoints(randomPoints(rng, 400, -6.0f, 6.0f));
  const int k = 5;
  const double gate_sq = 5.0;

  PointVec wide, tight;
  std::vector<float> wide_distances, tight_distances;
  int kept = 0, rejected = 0;
  for (int i = 0; i < 2000; ++i) {
    const PointType q = randomPoints(rng, 1, -6.0f, 6.0f)[0];
    map.nearestSearch(q, k, 5.0, wide, wide_distances);
    map.nearestSearch(q, k, std::sqrt(gate_sq), tight, tight_distances);
    const bool gate_passes =
      static_cast<int>(wide_distances.size()) >= k && wide_distances[k - 1] < gate_sq;
    if (gate_passes) {
      ++kept;
      ASSERT_EQ(tight_distances.size(), wide_distances.size());
      for (int j = 0; j < k; ++j) EXPECT_FLOAT_EQ(tight_distances[j], wide_distances[j]);
    } else {
      ++rejected;
      EXPECT_LT(static_cast<int>(tight_distances.size()), k);
    }
  }
  // Both branches exercised, or the test proves nothing.
  EXPECT_GT(kept, 100);
  EXPECT_GT(rejected, 100);
}

// Past capacity the grid longest without an insertion goes, whole; a grid
// inserted into again moves to the front and survives a younger one.
TEST(IVoxMap, EvictsTheLeastRecentlyInsertedGrid)
{
  IVoxMap map(1.0, kFineVoxel, 2, 6);
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

// The case the old outside-the-map rule missed: the voxel's point sits in its
// far corner and five points of the neighbouring voxels are nearer to the
// newcomer, so the newcomer's 5 nearest never include it. The map refuses the
// newcomer anyway, and the first point is the one it keeps. All of it inside
// grid (0, 0, 0), so the search sees every point.
TEST(IVoxMap, RefusesATakenVoxelWhoseFirstPointIsNotAmongTheNearest)
{
  IVoxMap map(kGrid, kVoxel, 1000, 6);
  const PointType first = makePoint(0.59f, 0.59f, 0.59f);  // voxel (1, 1, 1), far corner
  PointVec setup = {
    first,
    makePoint(0.29f, 0.31f, 0.31f),  // voxel (0, 1, 1)
    makePoint(0.31f, 0.29f, 0.31f),  // voxel (1, 0, 1)
    makePoint(0.31f, 0.31f, 0.29f),  // voxel (1, 1, 0)
    makePoint(0.29f, 0.29f, 0.31f),  // voxel (0, 0, 1)
    makePoint(0.29f, 0.31f, 0.29f)   // voxel (0, 1, 0)
  };
  map.addPoints(setup);
  ASSERT_EQ(map.numPoints(), 6u);

  const PointType newcomer = makePoint(0.31f, 0.31f, 0.31f);  // voxel (1, 1, 1), near corner
  PointVec nearest;
  std::vector<float> distances;
  map.nearestSearch(newcomer, 5, 5.0, nearest, distances);
  ASSERT_EQ(nearest.size(), 5u);
  for (const PointType & p : nearest) ASSERT_NE(p.x, first.x);  // the old rule's blind spot

  PointVec batch = {newcomer};
  map.addPoints(batch);
  EXPECT_EQ(map.numPoints(), 6u);
  EXPECT_EQ(map.numRefused(), 1u);
  map.nearestSearch(makePoint(0.58f, 0.58f, 0.58f), 1, 5.0, nearest, distances);
  ASSERT_EQ(nearest.size(), 1u);
  EXPECT_FLOAT_EQ(nearest[0].x, first.x);
}

// 2 m is not a multiple of 0.3 m, so the voxel [1.8, 2.1) spans two grids.
// It still holds one point: the second, in the other grid, is refused.
TEST(IVoxMap, OnePointPerVoxelAcrossAGridBoundary)
{
  IVoxMap map(kGrid, kVoxel, 1000, 6);
  PointVec first = {makePoint(1.85f, 0.1f, 0.1f)};   // grid 0
  PointVec second = {makePoint(2.05f, 0.1f, 0.1f)};  // grid 1, same voxel
  map.addPoints(first);
  map.addPoints(second);
  EXPECT_EQ(map.numPoints(), 1u);
  EXPECT_EQ(map.numGrids(), 1u);
  EXPECT_EQ(map.numRefused(), 1u);
}

// Duplicates inside one batch: the first in batch order stays.
TEST(IVoxMap, KeepsTheFirstOfAVoxelWithinOneBatch)
{
  IVoxMap map(kGrid, kVoxel, 1000, 6);
  PointVec batch = {
    makePoint(0.10f, 0.10f, 0.10f), makePoint(0.20f, 0.20f, 0.20f), makePoint(0.50f, 0.10f, 0.10f)};
  map.addPoints(batch);
  EXPECT_EQ(map.numPoints(), 2u);
  EXPECT_EQ(map.numRefused(), 1u);

  PointVec nearest;
  std::vector<float> distances;
  map.nearestSearch(makePoint(0.20f, 0.20f, 0.20f), 1, 5.0, nearest, distances);
  ASSERT_EQ(nearest.size(), 1u);
  EXPECT_FLOAT_EQ(nearest[0].x, 0.10f);
}

// An evicted grid gives its voxels back: a later point there is accepted.
TEST(IVoxMap, AnEvictedGridFreesItsVoxels)
{
  IVoxMap map(1.0, kVoxel, 1, 6);
  PointVec a = {makePoint(0.50f, 0.50f, 0.50f)};
  PointVec b = {makePoint(1.50f, 0.50f, 0.50f)};
  PointVec a_again = {makePoint(0.55f, 0.50f, 0.50f)};  // a's voxel
  map.addPoints(a);
  map.addPoints(b);  // a's grid evicted
  ASSERT_EQ(map.numEvicted(), 1u);
  map.addPoints(a_again);
  EXPECT_EQ(map.numRefused(), 0u);
  EXPECT_EQ(map.numPoints(), 1u);  // a_again in, b's grid evicted in turn
  EXPECT_EQ(map.numEvicted(), 2u);
}

// Stress: one wall seen in 300 scans of 600 points, with 2 cm range noise and
// a few millimetres of pose jitter, straddling a voxel boundary so the noise
// splits it over two layers. The map holds exactly one point per voxel the
// observations ever touched, never more, and the later scans add almost
// nothing: the growth that slowed the search on robot01 is gone.
TEST(IVoxMap, AWallSeenRepeatedlyHoldsOnePointPerVoxel)
{
  IVoxMap map(kGrid, kVoxel, 1000000, 6);
  std::mt19937 rng(3);
  std::normal_distribution<float> range_noise(0.0f, 0.02f);
  std::normal_distribution<float> pose_jitter(0.0f, 0.005f);
  std::uniform_real_distribution<float> along(0.0f, 4.0f);
  std::uniform_real_distribution<float> height(0.0f, 2.0f);
  const float wall_x = 1.2f;  // on the boundary between voxel layers 3 and 4

  std::set<VoxelIndex> touched;
  std::size_t inserted_total = 0;
  std::size_t points_after_half = 0;
  const int scans = 300;
  for (int scan = 0; scan < scans; ++scan) {
    const float jitter_x = pose_jitter(rng);
    const float jitter_y = pose_jitter(rng);
    PointVec batch;
    batch.reserve(600);
    for (int i = 0; i < 600; ++i) {
      const PointType p =
        makePoint(wall_x + jitter_x + range_noise(rng), along(rng) + jitter_y, height(rng));
      batch.push_back(p);
      touched.insert(voxelIndex(p, kVoxel));
    }
    map.addPoints(batch);
    inserted_total += batch.size();
    if (scan == scans / 2 - 1) points_after_half = map.numPoints();
  }
  EXPECT_EQ(map.numPoints(), touched.size());
  EXPECT_EQ(map.numPoints() + map.numRefused(), inserted_total);
  // Two layers of 14 x 7 voxels, plus the odd far-tail sample in a third.
  EXPECT_LE(map.numPoints(), 3u * 14u * 7u);
  const std::size_t second_half_growth = map.numPoints() - points_after_half;
  EXPECT_LE(second_half_growth * 50, map.numPoints());  // under 2 %
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
