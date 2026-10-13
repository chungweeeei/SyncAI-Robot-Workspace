// The geometry rules of OctomapPlanner3D's surface map and search, on the
// synthetic scenes of synthetic_octree.hpp. Each test is one promise a
// quadruped planner has to keep: a ramp is one piece of ground, a stair is
// not, a kerb is a wall, a wall is inflated by the robot radius, a hole in the
// observed floor is closed but a floor edge is not extended, two floors never
// connect, and a goal snaps to the floor the robot is on -- except over a
// stair the robot walked, which a stair link joins in both directions while
// a walked kerb stays a wall.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "syncai_global_planner/plugins/octomap_planner_3d/path_postprocess.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_astar.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/synthetic_octree.hpp"

using namespace syncai_global_planner::octomap_planner_3d;  // NOLINT

namespace
{

std::shared_ptr<const SurfaceMap> build(const std::string & scene, BuildStats * stats = nullptr)
{
  const auto tree = synthetic::makeScene(scene);
  EXPECT_NE(tree, nullptr) << scene;
  return buildSurfaceMap(*tree, SurfaceMapParams{}, stats);
}

// The traversable cell at (x, y) nearest in height to z, or kNoCell.
CellId cellAt(const SurfaceMap & m, double x, double y, double z)
{
  const int ix = m.ixOf(x), iy = m.iyOf(y);
  if (!m.inGrid(ix, iy)) {
    return kNoCell;
  }
  const int kz = static_cast<int>(std::floor((z - m.z0) / m.res)) - 1;
  const CellId c = m.layerNear(m.col(ix, iy), kz, 3);
  return (c != kNoCell && m.traversable(c)) ? c : kNoCell;
}

bool connected(
  const SurfaceMap & m, double x0, double y0, double z0, double x1, double y1, double z1)
{
  const CellId a = cellAt(m, x0, y0, z0), b = cellAt(m, x1, y1, z1);
  if (a == kNoCell || b == kNoCell) {
    return false;
  }
  return m.cells[a].component == m.cells[b].component;
}

}  // namespace

TEST(SurfaceMap, FlatFloorIsOneComponentInflatedAtTheEdge)
{
  BuildStats s;
  const auto m = build("flat", &s);
  EXPECT_EQ(s.components, 1u);
  // The grid edge is a drop-off: the outermost ring is step-rejected and the
  // next two (robot_radius 0.25 = 2 cells) are footprint-blocked.
  EXPECT_EQ(cellAt(*m, 0.05, 4.0, 0.0), kNoCell);
  EXPECT_EQ(cellAt(*m, 0.25, 4.0, 0.0), kNoCell);
  EXPECT_NE(cellAt(*m, 0.35, 4.0, 0.0), kNoCell);
  EXPECT_NEAR(m->zTop(cellAt(*m, 6.0, 4.0, 0.0)), 0.0, 1e-9);
}

TEST(SurfaceMap, RampIsTraversableAndContinuous)
{
  BuildStats s;
  const auto m = build("ramp", &s);
  EXPECT_EQ(s.components, 1u);
  EXPECT_EQ(s.rejected[static_cast<size_t>(Reject::Slope)], 0u);
  EXPECT_TRUE(connected(*m, 2.0, 4.0, 0.0, 10.0, 4.0, 4.0 * std::tan(15.0 * M_PI / 180.0)));
  // The cell half-way up sits at the ramp's height, not on the lower floor.
  const CellId mid = cellAt(*m, 6.0, 4.0, 2.0 * std::tan(15.0 * M_PI / 180.0));
  ASSERT_NE(mid, kNoCell);
  EXPECT_NEAR(m->zTop(mid), 2.0 * std::tan(15.0 * M_PI / 180.0), 0.15);
}

TEST(SurfaceMap, StairsSplitTheFloor)
{
  // Since single risers pass the step rule, the cells at a flight's risers
  // are traversable islands (a dog can stand on a step); the treads between
  // them, which see two risers in the disc, are not, so the two floors stay
  // apart and are the two largest components.
  BuildStats s;
  const auto m = build("stairs", &s);
  EXPECT_GE(s.components, 2u);
  EXPECT_FALSE(connected(*m, 2.0, 4.0, 0.0, 10.0, 4.0, 1.02));
  const CellId lo = cellAt(*m, 2.0, 4.0, 0.0), hi = cellAt(*m, 10.0, 4.0, 1.02);
  ASSERT_NE(lo, kNoCell);
  ASSERT_NE(hi, kNoCell);
  size_t n_lo = 0, n_hi = 0;
  for (CellId c = 0; c < m->cells.size(); ++c) {
    n_lo += m->traversable(c) && m->cells[c].component == m->cells[lo].component;
    n_hi += m->traversable(c) && m->cells[c].component == m->cells[hi].component;
  }
  EXPECT_EQ(std::min(n_lo, n_hi), s.component_sizes[1]);
}

TEST(SurfaceMap, KerbIsOneStepButADropSplitsTheFloor)
{
  // 0.15 m is within max_step_height (0.17): one piece of ground, the kerb
  // crossed as a single step. 0.5 m is a ledge: two components.
  BuildStats s;
  const auto m = build("kerb", &s);
  EXPECT_EQ(s.components, 1u);
  EXPECT_TRUE(connected(*m, 3.0, 4.0, 0.0, 9.0, 4.0, 0.15));
  BuildStats sd;
  const auto d = build("drop", &sd);
  EXPECT_EQ(sd.components, 2u);
  EXPECT_FALSE(connected(*d, 3.0, 4.0, 0.0, 9.0, 4.0, 0.5));
}

TEST(SurfaceMap, GapsInTheFootprintAreToleratedUpToAShare)
{
  // A 1.5 x 1.5 m void in the floor (too wide for the hole fill): walking
  // toward it along y = 3.75, the last traversable cells before the void have
  // gaps under their disc and cost more, the first blocked cell is blocked
  // for its gaps, and the old rule (every column) blocks earlier.
  synthetic::SceneBuilder b(0.1);
  b.flatFloor(0, 12, 0, 8, 0.0);
  b.erase(3.0, 4.5, 3.0, 4.5, -0.2, 0.2);
  const auto tree = b.finish();
  SurfaceMapParams strict;
  strict.footprint_min_support = 1.0;
  const auto m = buildSurfaceMap(*tree, SurfaceMapParams{});
  const auto ms = buildSurfaceMap(*tree, strict);
  SurfaceAStar astar(m);
  const double plain = astar.cellCostFactor(cellAt(*m, 1.0, 3.75, 0.0), SearchParams{});
  double first_blocked = -1, first_blocked_strict = -1;
  bool saw_gap_cell = false;
  for (double x = 1.05; x < 3.5; x += 0.1) {
    const CellId c = cellAt(*m, x, 3.75, 0.0);
    if (c == kNoCell) {
      first_blocked = x;
      const CellId raw = m->layerNear(m->col(m->ixOf(x), m->iyOf(3.75)), -1, 3);
      ASSERT_NE(raw, kNoCell);
      EXPECT_EQ(static_cast<Reject>(m->cells[raw].reject), Reject::FootprintGap);
      break;
    }
    if (m->cells[c].gap_cnt > 0) {
      saw_gap_cell = true;
      EXPECT_GT(astar.cellCostFactor(c, SearchParams{}), plain);
    }
  }
  for (double x = 1.05; x < 3.5; x += 0.1) {
    if (cellAt(*ms, x, 3.75, 0.0) == kNoCell) {
      first_blocked_strict = x;
      break;
    }
  }
  EXPECT_TRUE(saw_gap_cell);
  EXPECT_GT(first_blocked, 0.0);
  EXPECT_GT(first_blocked_strict, 0.0);
  EXPECT_LT(first_blocked_strict, first_blocked - 0.05);
  // A wall is never a gap: the "wall" scene's inflation is unchanged.
  const auto w = build("wall");
  EXPECT_EQ(cellAt(*w, 5.85, 6.0, 0.0), kNoCell);
  EXPECT_EQ(cellAt(*w, 5.75, 6.0, 0.0), kNoCell);
  EXPECT_NE(cellAt(*w, 5.55, 6.0, 0.0), kNoCell);
}

TEST(SurfaceAStar, TwoFloorsSnapSeparatelyAndNeverConnect)
{
  BuildStats s;
  const auto m = build("two_floors", &s);
  EXPECT_EQ(s.components, 2u);
  SurfaceAStar astar(m);
  SearchParams p;
  // A goal given at the robot's lidar height (0.48 above the floor) snaps to
  // that floor; the same xy with z 5.48 to the other.
  const SnapResult lo = astar.snap(6.0, 4.0, 0.48, p);
  const SnapResult hi = astar.snap(6.0, 4.0, 5.48, p);
  ASSERT_NE(lo.cell, kNoCell);
  ASSERT_NE(hi.cell, kNoCell);
  EXPECT_NEAR(m->zTop(lo.cell), 0.0, 1e-9);
  EXPECT_NEAR(m->zTop(hi.cell), 5.0, 1e-9);
  EXPECT_NE(m->cells[lo.cell].component, m->cells[hi.cell].component);
  // Preferring the start's component refuses the other floor and says so.
  const SnapResult refused = astar.snap(6.0, 4.0, 5.48, p, m->cells[lo.cell].component);
  EXPECT_EQ(refused.cell, kNoCell);
  EXPECT_NE(refused.detail.find("component"), std::string::npos);
  const PlanResult r = astar.plan(lo.cell, hi.cell, p);
  EXPECT_EQ(r.failure, PlanFailure::Disconnected);
}

TEST(SurfaceAStar, PlansThroughTheDoorAndKeepsClearance)
{
  const auto m = build("wall");
  SurfaceAStar astar(m);
  SearchParams p;
  const SnapResult a = astar.snap(3.0, 6.0, 0.0, p);
  const SnapResult b = astar.snap(9.0, 6.0, 0.0, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r = astar.plan(a.cell, b.cell, p);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  ASSERT_GE(r.cells.size(), 2u);
  EXPECT_EQ(r.cells.front(), a.cell);
  EXPECT_EQ(r.cells.back(), b.cell);
  // Every cell is traversable, consecutive cells are neighbours, and the
  // route crosses x = 6 inside the door.
  bool crossed = false;
  for (size_t i = 0; i < r.cells.size(); ++i) {
    EXPECT_TRUE(m->traversable(r.cells[i]));
    double x, y;
    m->cellXY(r.cells[i], x, y);
    if (std::fabs(x - 6.05) < 0.06) {
      crossed = true;
      EXPECT_GT(y, 3.6);
      EXPECT_LT(y, 4.4);
    }
    if (i > 0) {
      double px, py;
      m->cellXY(r.cells[i - 1], px, py);
      EXPECT_LE(std::hypot(x - px, y - py), m->res * std::sqrt(2.0) + 1e-9);
    }
  }
  EXPECT_TRUE(crossed);
  // The shortcut keeps the ends, stays traversable along every segment and
  // never comes closer to the boundary than the search did.
  const auto chain = shortcutPath(*m, r.cells);
  EXPECT_LT(chain.size(), r.cells.size());
  EXPECT_EQ(chain.front(), a.cell);
  EXPECT_EQ(chain.back(), b.cell);
  float min_clear = 1e9f;
  for (CellId c : r.cells) {
    min_clear = std::min(min_clear, m->cells[c].clearance_d);
  }
  for (size_t i = 1; i < chain.size(); ++i) {
    EXPECT_TRUE(lineOfSight(*m, chain[i - 1], chain[i], min_clear - 1e-4f));
  }
  const auto path = resamplePath(*m, chain, 0.1);
  ASSERT_GE(path.size(), 2u);
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 0.1 + 1e-6);
    EXPECT_NEAR(path[i].z, 0.0, 1e-9);
  }
}

TEST(SurfaceAStar, RampPathFollowsTheSurface)
{
  const auto m = build("ramp");
  SurfaceAStar astar(m);
  SearchParams p;
  const double top = 4.0 * std::tan(15.0 * M_PI / 180.0);
  const SnapResult a = astar.snap(2.0, 4.0, 0.0, p);
  const SnapResult b = astar.snap(10.0, 4.0, top, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r = astar.plan(a.cell, b.cell, p);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  const auto path = resamplePath(*m, shortcutPath(*m, r.cells), 0.1);
  double zmax = -1;
  for (size_t i = 1; i < path.size(); ++i) {
    zmax = std::max(zmax, path[i].z);
    EXPECT_LE(std::fabs(path[i].z - path[i - 1].z), 0.1 + 1e-9);  // one step per 0.1 m at most
  }
  EXPECT_NEAR(zmax, top, 0.15);
}

// The track a robot leaves walking along y = 4 from x0 to x1 on a scene whose
// ground height is height(x): one ground point every 0.25 m.
template <typename H>
std::vector<Waypoint> walk(double x0, double x1, H height)
{
  std::vector<Waypoint> t;
  for (double x = x0; x <= x1 + 1e-9; x += 0.25) {
    t.push_back({x, 4.0, height(x)});
  }
  return t;
}

TEST(StairLinks, AWalkedStairConnectsTheFloorsBothWays)
{
  const auto m = build("stairs");
  const auto stairs = [](double x) {
    return x < 4.0 ? 0.0 : 0.17 * std::min(6, static_cast<int>(std::floor((x - 4.0) / 0.29)) + 1);
  };
  const auto links = buildStairLinks(*m, walk(2.0, 10.0, stairs), StairLinkParams{});
  ASSERT_EQ(links->links.size(), 1u);
  const StairLink & l = links->links[0];
  // The first riser is one step from the floor and belongs to it, so the
  // link may start on it.
  EXPECT_LE(m->zTop(l.a), 0.2 + 1e-9);
  EXPECT_NEAR(m->zTop(l.b), 1.02, 0.05);  // the top step, quantised to the 0.1 m grid
  EXPECT_EQ(links->groupOf(m->cells[l.a].component), links->groupOf(m->cells[l.b].component));

  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult lo = astar.snap(2.0, 4.0, 0.0, p);
  ASSERT_NE(lo.cell, kNoCell);
  // The upper floor is reachable now, so preferring the start's component
  // no longer refuses it.
  const SnapResult hi = astar.snap(10.0, 4.0, 1.02, p, m->cells[lo.cell].component);
  ASSERT_NE(hi.cell, kNoCell) << hi.detail;
  for (const bool up : {true, false}) {
    const PlanResult r = up ? astar.plan(lo.cell, hi.cell, p) : astar.plan(hi.cell, lo.cell, p);
    ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
    ASSERT_EQ(r.links.size(), 1u);
    EXPECT_EQ(r.links[0].reversed, !up);
    const auto path = buildPath(*m, astar.links(), r.cells, r.links, true, 0.1);
    ASSERT_GE(path.size(), 2u);
    EXPECT_NEAR(path.front().z, up ? 0.0 : 1.02, 0.25);
    EXPECT_NEAR(path.back().z, up ? 1.02 : 0.0, 0.25);
    for (size_t i = 1; i < path.size(); ++i) {
      // Continuous: no jump between the floor stretches and the flight, and
      // the height only ever moves one way.
      EXPECT_LE(std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 0.1 + 1e-6);
      EXPECT_GE((up ? 1 : -1) * (path[i].z - path[i - 1].z), -1e-9);
    }
  }
}

TEST(StairLinks, AWalkedKerbMakesNoLinkAndAnUnwalkedDropIsReported)
{
  // The kerb is one piece of ground already: walking it makes no link, and
  // the plan over it uses none. The drop, never walked, stays two components
  // and the notes say so.
  const auto m = build("kerb");
  const auto links = buildStairLinks(
    *m, walk(3.0, 9.0, [](double x) { return x < 6.0 ? 0.0 : 0.15; }), StairLinkParams{});
  EXPECT_TRUE(links->links.empty());
  EXPECT_TRUE(links->notes.empty());
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult a = astar.snap(3.0, 4.0, 0.0, p);
  const SnapResult b = astar.snap(9.0, 4.0, 0.15, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r = astar.plan(a.cell, b.cell, p);
  EXPECT_EQ(r.failure, PlanFailure::None) << r.detail;
  EXPECT_TRUE(r.links.empty());

  const auto d = build("drop");
  const auto none =
    buildStairLinks(*d, walk(2.0, 5.0, [](double) { return 0.0; }), StairLinkParams{});
  EXPECT_TRUE(none->links.empty());
  ASSERT_EQ(none->notes.size(), 1u);
  EXPECT_NE(none->notes[0].find("not joined"), std::string::npos);
}

TEST(SurfaceAStar, NoCornerCutting)
{
  // Two blocked cells touching corner to corner must not be passable
  // diagonally. A wall across the whole floor with one diagonal gap is the
  // cleanest way to get that: without the rule the floor is one component.
  synthetic::SceneBuilder b(0.1);
  b.flatFloor(0, 12, 0, 8, 0.0);
  b.box(6.0, 6.1, 0.0, 4.0, 0.0, 1.0);
  b.box(6.1, 6.2, 4.1, 8.0, 0.0, 1.0);
  const auto tree = b.finish();
  BuildStats s;
  const auto m = buildSurfaceMap(*tree, SurfaceMapParams{}, &s);
  EXPECT_EQ(s.components, 2u);
}
