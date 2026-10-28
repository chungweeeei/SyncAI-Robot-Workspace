// The geometry rules of OctomapPlanner3D's surface map and search, on the
// synthetic scenes of synthetic_octree.hpp. Each test is one promise a
// quadruped planner has to keep: a ramp is one piece of ground, a stair is
// not, a kerb is one step, a wall is inflated by the robot radius, a hole in
// the observed floor is closed but a floor edge is not extended, two floors
// never connect, and a goal snaps to the floor the robot is on -- except
// over a stair the robot walked, which a stair link joins in both directions
// while a walked kerb makes no stair link. The dynamic overlay's and the
// keepout zones' cases follow, each introduced by its own banner, and the
// kerbs' (2026-10-29) last: a kerb is crossed square on a kerb link and
// never run along.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
#include "syncai_global_planner/plugins/octomap_planner_3d/keepout_zones.hpp"
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

TEST(SurfaceAStar, AHintOffTheOnlyLayerSnapsToItButOverlappingFloorsStillNeedOne)
{
  // The ramp's top is 1.07 m up: a goal there sent at z = 0, as the backend
  // sends every goal, is beyond snap_z_window, and the one layer under it is
  // the one meant.
  const auto r = build("ramp");
  SurfaceAStar ra(r);
  SearchParams p;
  const double top = 4.0 * std::tan(15.0 * M_PI / 180.0);
  ASSERT_GT(top, p.snap_z_window);
  const SnapResult start = ra.snap(2.0, 4.0, 0.0, p);
  ASSERT_NE(start.cell, kNoCell);
  EXPECT_FALSE(start.z_fallback);
  const SnapResult up = ra.snap(10.0, 4.0, 0.0, p, r->cells[start.cell].component);
  ASSERT_NE(up.cell, kNoCell) << up.detail;
  EXPECT_TRUE(up.z_fallback);
  EXPECT_NEAR(r->zTop(up.cell), top, 0.15);
  EXPECT_NEAR(up.dz, r->zTop(up.cell), 1e-9);
  p.snap_single_layer = false;
  EXPECT_EQ(ra.snap(10.0, 4.0, 0.0, p).cell, kNoCell);
  p.snap_single_layer = true;

  // Two floors 5 m apart: a hint half-way is in neither window, and the
  // fallback must not pick one for it.
  const auto f = build("two_floors");
  SurfaceAStar fa(f);
  const SnapResult mid = fa.snap(6.0, 4.0, 2.5, p);
  EXPECT_EQ(mid.cell, kNoCell);
  EXPECT_FALSE(mid.z_fallback);
  EXPECT_NE(mid.detail.find("more than one floor"), std::string::npos) << mid.detail;
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
  // One walked flight, laid as a row of straight links across the stair's
  // whole 8 m width (the walk was along y = 4, the entries reach 2 m either
  // side of it): the track found the stair, the surfaces gave its direction.
  ASSERT_EQ(links->flights.size(), 1u);
  const StairFlight & f = links->flights[0];
  EXPECT_EQ(f.count, links->links.size());
  EXPECT_GE(f.count, 20u);
  EXPECT_NEAR(f.heading_deg, 0.0, 10.0);  // the scene's stair climbs along +x
  EXPECT_GE(f.width, 3.5);
  for (const StairLink & l : links->links) {
    EXPECT_TRUE(l.straight);
    ASSERT_EQ(l.pts.size(), 2u);
    // The first riser is one step from the floor and belongs to it, so a
    // link may start on it, and the top step is where the upper floor begins.
    EXPECT_LE(m->zTop(l.a), 0.2 + 1e-9);
    EXPECT_GE(m->zTop(l.b), 0.8);
    EXPECT_NEAR(l.pts.front().y, l.pts.back().y, 0.15);  // along the uphill direction
    EXPECT_EQ(links->groupOf(m->cells[l.a].component), links->groupOf(m->cells[l.b].component));
  }
  const StairLink & l = links->links[0];

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

TEST(StairLinks, AStraightFlightIsEnteredWhereTheRouteMeetsIt)
{
  // The robot walked the stair along y = 4; a route along y = 5.5 climbs it
  // there, straight, instead of detouring to the walked line and back (which
  // is what the walked link alone makes it do -- the 2026-10-14 behaviour,
  // kept under stair_straight: false, and what dp showed as a 27 deg turn at
  // the stair head).
  const auto m = build("stairs");
  const auto stairs = [](double x) {
    return x < 4.0 ? 0.0 : 0.17 * std::min(6, static_cast<int>(std::floor((x - 4.0) / 0.29)) + 1);
  };
  const auto track = walk(2.0, 10.0, stairs);
  for (const bool straight : {true, false}) {
    StairLinkParams sp;
    sp.straight = straight;
    const auto links = buildStairLinks(*m, track, sp);
    SurfaceAStar astar(m, links);
    SearchParams p;
    const SnapResult lo = astar.snap(2.0, 5.5, 0.0, p);
    const SnapResult hi = astar.snap(10.0, 5.5, 1.02, p, m->cells[lo.cell].component);
    ASSERT_NE(lo.cell, kNoCell);
    ASSERT_NE(hi.cell, kNoCell) << hi.detail;
    const PlanResult r = astar.plan(lo.cell, hi.cell, p);
    ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
    ASSERT_EQ(r.links.size(), 1u);
    const auto path = buildPath(*m, astar.links(), r.cells, r.links, true, 0.1);
    double ymin = 1e9, worst_turn = 0.0, prev_h = 0.0;
    bool have_h = false;
    for (size_t i = 0; i < path.size(); ++i) {
      ymin = std::min(ymin, path[i].y);
      if (i == 0 || std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) < 0.02) {
        continue;
      }
      const double h = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
      if (have_h) {
        worst_turn = std::max(worst_turn, std::fabs(std::remainder(h - prev_h, 2 * M_PI)));
      }
      prev_h = h;
      have_h = true;
    }
    if (straight) {
      EXPECT_GE(ymin, 5.3) << "the route left y = 5.5 to reach the stair";
      EXPECT_FALSE(links->links[r.links[0].link].pts.size() > 2);
      // The shortcut spans the link (stairCrossing): one line from floor to
      // floor, no bend where the link's cell-centre ends used to be.
      EXPECT_LT(worst_turn * 180.0 / M_PI, 5.0);
      EXPECT_GE(path.back().z - path.front().z, 0.8);
    } else {
      EXPECT_LE(ymin, 4.2) << "the walked link alone should have forced the detour to y = 4";
    }
  }
}

TEST(StairLinks, AStraightCrossingKeepsTheFarFloorsOwnHeight)
{
  // dp, 2026-10-22: a route up 2F's step, shortcut across it in one line
  // (stairCrossing), had one pose 0.6 m under the floor 3.7 m past the top.
  // The floor there had risen 0.3 m above the step's top, and the samples
  // took the surface nearest the top's height -- in a column with a rejected
  // layer under the floor, that layer. MPPI read the dip as a stair going
  // down and walked the rest of the way backwards. Here: the stairs scene's
  // flight, an upper floor that rises 0.3 m more in 0.1 m steps (one piece of
  // ground) over the next 3 m, and one occupied voxel 0.5 m under it on the
  // route.
  const auto ground = [](double x) {
    if (x < 4.0) {
      return 0.0;
    }
    if (x < 5.74) {
      return 0.17 * (std::floor((x - 4.0) / 0.29) + 1);
    }
    return 1.02 + 0.1 * std::min(3.0, std::floor((x - 5.74) / 1.0));
  };
  synthetic::SceneBuilder b;
  b.floor(0, 12.0, 0, 8.0, [&](double x, double) { return ground(x); });
  b.box(9.5, 9.6, 5.5, 5.6, 0.7, 0.8);
  const auto m = buildSurfaceMap(*b.finish(), SurfaceMapParams{});
  const auto links = buildStairLinks(*m, walk(2.0, 10.0, ground), StairLinkParams{});
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult lo = astar.snap(2.0, 5.55, 0.0, p);
  ASSERT_NE(lo.cell, kNoCell);
  const SnapResult hi = astar.snap(11.0, 5.55, 1.32, p, m->cells[lo.cell].component);
  ASSERT_NE(hi.cell, kNoCell) << hi.detail;
  const PlanResult r = astar.plan(lo.cell, hi.cell, p);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  ASSERT_EQ(r.links.size(), 1u);
  const auto path = buildPath(*m, astar.links(), r.cells, r.links, true, 0.1);
  ASSERT_GE(path.size(), 2u);
  // The crossing is what is under test: one line, floor to floor.
  double worst_turn = 0.0, prev_h = 0.0;
  bool have_h = false, past_ghost = false;
  for (size_t i = 1; i < path.size(); ++i) {
    EXPECT_GE(path[i].z - path[i - 1].z, -1e-9) << "a dip at x " << path[i].x;
    past_ghost = past_ghost || path[i].x > 9.6;
    if (std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) < 0.02) {
      continue;
    }
    const double h = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
    if (have_h) {
      worst_turn = std::max(worst_turn, std::fabs(std::remainder(h - prev_h, 2 * M_PI)));
    }
    prev_h = h;
    have_h = true;
  }
  EXPECT_LT(worst_turn * 180.0 / M_PI, 5.0);
  EXPECT_TRUE(past_ghost);
  EXPECT_NEAR(path.back().z, 1.32, 0.15);
}

TEST(StairLinks, AWalkedKerbMakesNoStairLinkAndAnUnwalkedDropIsReported)
{
  // The kerb is one piece of ground already: walking it makes no stair
  // link (its kerb links are the kerb's own, below), and the plan over it
  // takes one kerb link, nothing else. The drop, never walked, stays two
  // components and the notes say so.
  const auto m = build("kerb");
  const auto links = buildStairLinks(
    *m, walk(3.0, 9.0, [](double x) { return x < 6.0 ? 0.0 : 0.15; }), StairLinkParams{});
  for (const StairLink & l : links->links) {
    EXPECT_TRUE(l.kerb);
  }
  EXPECT_TRUE(links->notes.empty());
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult a = astar.snap(3.0, 4.0, 0.0, p);
  const SnapResult b = astar.snap(9.0, 4.0, 0.15, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r = astar.plan(a.cell, b.cell, p);
  EXPECT_EQ(r.failure, PlanFailure::None) << r.detail;
  ASSERT_EQ(r.links.size(), 1u);
  EXPECT_TRUE(links->links[r.links[0].link].kerb);

  const auto d = build("drop");
  const auto none =
    buildStairLinks(*d, walk(2.0, 5.0, [](double) { return 0.0; }), StairLinkParams{});
  EXPECT_TRUE(none->links.empty());
  ASSERT_EQ(none->notes.size(), 1u);
  EXPECT_NE(none->notes[0].find("not joined"), std::string::npos);
}

TEST(StairLinks, AStairNobodyWalkedIsDetectedAndADropIsNot)
{
  // No track at all: the stairs scene's flight is found off the map and laid
  // as straight links across its width, as the walked one is; the drop (one
  // 0.5 m ledge) makes no link, and the kerb (one piece of ground) no stair
  // link -- and no kerb link either when those are off.
  const auto m = build("stairs");
  const auto links = buildStairLinks(*m, {}, StairLinkParams{});
  ASSERT_EQ(links->flights.size(), 1u);
  EXPECT_TRUE(links->flights[0].detected);
  EXPECT_NEAR(links->flights[0].heading_deg, 0.0, 10.0);
  // Entries are tried within stair_straight_half_width (2 m) of the
  // stand-in walk's foot, as of a walked one's, and the stand-in may cross
  // anywhere more than half a metre in from the edges of this 8 m flight.
  EXPECT_GE(links->flights[0].width, 2.0);
  for (const StairLink & l : links->links) {
    EXPECT_TRUE(l.detected);
    EXPECT_TRUE(l.straight);
  }
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult lo = astar.snap(2.0, 4.0, 0.0, p);
  ASSERT_NE(lo.cell, kNoCell);
  const SnapResult hi = astar.snap(10.0, 4.0, 1.02, p, m->cells[lo.cell].component);
  ASSERT_NE(hi.cell, kNoCell) << hi.detail;
  const PlanResult r = astar.plan(hi.cell, lo.cell, p);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  EXPECT_EQ(r.links.size(), 1u);

  StairLinkParams off;
  off.detect = false;
  EXPECT_TRUE(buildStairLinks(*m, {}, off)->links.empty());
  EXPECT_TRUE(buildStairLinks(*build("drop"), {}, StairLinkParams{})->links.empty());
  StairLinkParams no_kerbs;
  no_kerbs.kerb_links = false;
  EXPECT_TRUE(buildStairLinks(*build("kerb"), {}, no_kerbs)->links.empty());
}

TEST(StairLinks, AnUnwalkedUStairIsLaidAsFlightsAndCorners)
{
  // dp's middle stair in small (2026-10-25): two flights of six 0.17 / 0.29
  // steps side by side, a wall between them, joined by a landing at the far
  // end -- up +x along y 0.5..1.7, back -x along y 2.3..3.5 -- from a ground
  // floor at x < 3 to an upper floor at 2.04 over x < 3, y > 2.3. The
  // shortest way through cuts the landing's inside corner round the wall's
  // end; the stand-in walk has to cross the landing for the U to be read.
  synthetic::SceneBuilder b;
  b.flatFloor(0, 3.0, 0, 8.0, 0.0);
  b.floor(3.0, 4.74, 0.5, 1.7, [](double x, double) {
    return 0.17 * std::min(6, static_cast<int>(std::floor((x - 3.0) / 0.29)) + 1);
  });
  b.flatFloor(4.74, 6.0, 0.5, 3.5, 1.02);
  b.floor(3.0, 4.74, 2.3, 3.5, [](double x, double) {
    return 1.02 + 0.17 * std::min(6, static_cast<int>(std::floor((4.74 - x) / 0.29)) + 1);
  });
  b.flatFloor(0, 3.0, 2.3, 8.0, 2.04);
  b.box(3.0, 4.74, 1.7, 2.3, 0.0, 3.0);  // the wall between the flights
  b.box(3.0, 6.1, 0.4, 0.5, 0.0, 3.5);   // the well's walls
  b.box(3.0, 6.1, 3.5, 3.6, 0.0, 3.5);
  b.box(6.0, 6.1, 0.4, 3.6, 0.0, 3.5);
  const auto m = buildSurfaceMap(*b.finish(), SurfaceMapParams{});
  const auto links = buildStairLinks(*m, {}, StairLinkParams{});
  ASSERT_EQ(links->links.size(), 1u) << (links->notes.empty() ? "" : links->notes[0]);
  const StairLink & l = links->links[0];
  EXPECT_TRUE(l.detected);
  ASSERT_TRUE(l.flights) << l.geometry;
  EXPECT_EQ(l.n_flights, 2u) << l.geometry;
  EXPECT_EQ(l.turns.size(), 2u) << l.geometry;
  // The two corners are on the landing, past the wall's end.
  for (const size_t t : l.turns) {
    EXPECT_GT(l.pts[t].x, 4.74) << l.geometry;
    EXPECT_NEAR(l.pts[t].z, 1.02, 0.25) << l.geometry;
  }
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult lo = astar.snap(1.5, 1.0, 0.0, p);
  ASSERT_NE(lo.cell, kNoCell);
  const SnapResult hi = astar.snap(1.5, 5.0, 2.04, p, m->cells[lo.cell].component);
  ASSERT_NE(hi.cell, kNoCell) << hi.detail;
  const PlanResult r = astar.plan(lo.cell, hi.cell, p);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  EXPECT_EQ(r.links.size(), 1u);
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

// ---- the dynamic overlay (dynamic_overlay.hpp, 2026-10-21) ----
//
// What a 3D map's "path blocked -> replan round it" rests on: a return in the
// band above a surface marks that surface's cell and no other floor's, a ray
// through the band clears it, marks age out, the dilation stays on the floor
// and within reach, the robot's own disc is exempt from the dilation but not
// from a mark, the search and the shortcut keep off the marks and pay for
// the band beyond, a closed door is no_path (not disconnected), a goal under
// a person does not snap, and a stair link is taken with the floor stretch
// bent round the mark.

namespace
{

// The lidar 0.48 m over the floor at (x, y): where pointlio_body is.
Waypoint sensorOver(double x, double y, double floor_z)
{
  return {x, y, floor_z + 0.48};
}

// Traversable cells of the floor at z the mask blocks.
size_t blockedOnFloor(const SurfaceMap & m, const DynamicMask & mask, double z)
{
  size_t n = 0;
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.traversable(c) && std::fabs(m.zTop(c) - z) < 0.3 && mask.d[c] <= mask.block_radius) {
      ++n;
    }
  }
  return n;
}

BlockedView viewFrom(const DynamicMask & mask, CellId start)
{
  BlockedView v;
  v.mask = &mask;
  v.exempt = start;
  v.exempt_radius = mask.block_radius;
  return v;
}

}  // namespace

TEST(DynamicOverlay, MarksTheSurfaceUnderTheReturnAndNoOtherFloor)
{
  const auto m = build("two_floors");
  DynamicOverlay o;
  o.setMap(m);
  // A knee 2 m ahead of the sensor on the ground floor: a mark on that
  // floor's cell and nothing on the floor 5 m above it.
  o.addScan({{5.05, 4.05, 0.3}}, sensorOver(3.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  EXPECT_EQ(mask->marked, 1u);
  const CellId lo = cellAt(*m, 5.05, 4.05, 0.0), hi = cellAt(*m, 5.05, 4.05, 5.0);
  ASSERT_NE(lo, kNoCell);
  ASSERT_NE(hi, kNoCell);
  EXPECT_FLOAT_EQ(mask->d[lo], 0.0f);
  EXPECT_TRUE(std::isinf(mask->d[hi]));
  EXPECT_GT(blockedOnFloor(*m, *mask, 0.0), 0u);
  EXPECT_EQ(blockedOnFloor(*m, *mask, 5.0), 0u);
  // The same knee upstairs marks the upper floor only.
  o.clear();
  o.addScan({{5.05, 4.05, 5.3}}, sensorOver(3.05, 4.05, 5.0), 1.0);
  const auto up = o.snapshot(1.0);
  ASSERT_NE(up, nullptr);
  EXPECT_TRUE(std::isinf(up->d[lo]));
  EXPECT_FLOAT_EQ(up->d[hi], 0.0f);
  // layerUnder is the surface a point lies on, not the nearest one in
  // height: a point just under the upper floor's slab is the lower floor's.
  const uint32_t col = m->col(m->ixOf(5.05), m->iyOf(4.05));
  EXPECT_EQ(DynamicOverlay::layerUnder(*m, col, 0.3), lo);
  EXPECT_EQ(DynamicOverlay::layerUnder(*m, col, 4.8), lo);
  EXPECT_EQ(DynamicOverlay::layerUnder(*m, col, 5.3), hi);
}

TEST(DynamicOverlay, ReturnsOutsideTheBandOrInTheWallRingOrOnTheRobotDoNotMark)
{
  const auto flat = build("flat");
  DynamicOverlay o;
  o.setMap(flat);
  const Waypoint sensor = sensorOver(3.05, 4.05, 0.0);
  o.addScan({{5.05, 4.05, 0.05}}, sensor, 0.0);  // the floor itself
  o.addScan({{5.05, 4.05, 0.8}}, sensor, 0.1);   // above the body
  o.addScan({{3.45, 4.05, 0.3}}, sensor, 0.2);   // the robot's own leg (self_radius)
  EXPECT_EQ(o.snapshot(0.2), nullptr);
  EXPECT_EQ(o.hotCount(), 0u);
  // A wall's returns land in the ring the footprint rule cut, which is
  // never marked: inflating it would narrow every corridor and close every
  // door under ~0.9 m whenever a wall is in range.
  const auto wall = build("wall");
  o.setMap(wall);
  const Waypoint in_front = sensorOver(4.05, 3.05, 0.0);
  o.addScan({{5.95, 3.05, 0.3}, {6.05, 3.05, 0.3}, {5.85, 3.05, 0.3}}, in_front, 0.0);
  EXPECT_EQ(o.snapshot(0.0), nullptr);
  // One step out of the ring, on floor the robot may stand on, it marks.
  o.addScan({{5.45, 3.05, 0.3}}, in_front, 0.1);
  EXPECT_NE(o.snapshot(0.1), nullptr);
}

TEST(DynamicOverlay, ARayThroughTheBandClearsAMarkAndAReturnKeepsItsOwnCell)
{
  const auto m = build("flat");
  DynamicOverlay o;
  o.setMap(m);
  const Waypoint sensor = sensorOver(3.05, 4.05, 0.0);
  const CellId near = cellAt(*m, 4.55, 4.05, 0.0), far = cellAt(*m, 5.45, 4.05, 0.0);
  o.addScan({{4.55, 4.05, 0.3}}, sensor, 0.0);
  ASSERT_NE(o.snapshot(0.0), nullptr);
  // A return short of the mark leaves it: its ray ends before the column.
  // (Above the band itself, so it is a ray and not a second mark.)
  o.addScan({{4.05, 4.05, 0.8}}, sensor, 0.1);
  auto mask = o.snapshot(0.1);
  ASSERT_NE(mask, nullptr);
  EXPECT_FLOAT_EQ(mask->d[near], 0.0f);
  EXPECT_EQ(mask->marked, 1u);
  // A wall point at 1.5 m past the mark leaves it too: over the mark that
  // ray is still 1.1 m up, above the band. It marks nothing itself.
  o.addScan({{5.45, 4.05, 1.5}}, sensor, 0.2);
  mask = o.snapshot(0.2);
  ASSERT_NE(mask, nullptr);
  EXPECT_FLOAT_EQ(mask->d[near], 0.0f);
  EXPECT_EQ(mask->marked, 1u);
  // A low return past the mark, whose ray crosses the band over it (0.37 m
  // up at the mark's column): the mark is seen empty, and the return marks
  // its own cell.
  o.addScan({{5.45, 4.05, 0.3}}, sensor, 0.3);
  mask = o.snapshot(0.3);
  ASSERT_NE(mask, nullptr);
  EXPECT_EQ(mask->marked, 1u);
  EXPECT_GT(mask->d[near], mask->block_radius);  // cleared: in the new mark's band, not its ring
  EXPECT_FLOAT_EQ(mask->d[far], 0.0f);
  // Seen again from the same place: a ray never clears its own return's
  // column, so the mark stays and there is still exactly one.
  o.addScan({{5.45, 4.05, 0.3}}, sensor, 0.4);
  mask = o.snapshot(0.4);
  ASSERT_NE(mask, nullptr);
  EXPECT_FLOAT_EQ(mask->d[far], 0.0f);
  EXPECT_EQ(mask->marked, 1u);
}

TEST(DynamicOverlay, MarksExpireAndClearAndANewMapDropThem)
{
  const auto m = build("flat");
  DynamicOverlay o;
  o.setMap(m);
  const Waypoint sensor = sensorOver(3.05, 4.05, 0.0);
  o.addScan({{5.05, 4.05, 0.3}}, sensor, 100.0);
  EXPECT_NE(o.snapshot(129.0), nullptr);  // obstacle_memory is 30 s
  EXPECT_EQ(o.hotCount(), 1u);
  EXPECT_EQ(o.snapshot(131.0), nullptr);
  EXPECT_EQ(o.hotCount(), 0u);
  // A refreshed mark lives on from its last sighting.
  o.addScan({{5.05, 4.05, 0.3}}, sensor, 140.0);
  o.addScan({{5.05, 4.05, 0.3}}, sensor, 160.0);
  EXPECT_NE(o.snapshot(189.0), nullptr);
  EXPECT_EQ(o.clear(), 1u);
  EXPECT_EQ(o.snapshot(189.0), nullptr);
  // A new map (a reload) starts empty, and the mask is sized to it.
  o.addScan({{5.05, 4.05, 0.3}}, sensor, 200.0);
  const auto two = build("two_floors");
  o.setMap(two);
  EXPECT_EQ(o.snapshot(200.0), nullptr);
  o.addScan({{5.05, 4.05, 0.3}}, sensor, 201.0);
  const auto mask = o.snapshot(201.0);
  ASSERT_NE(mask, nullptr);
  EXPECT_EQ(mask->d.size(), two->cells.size());
}

TEST(DynamicOverlay, DilationStaysOnTheFloorAndWithinReach)
{
  const auto m = build("two_floors");
  DynamicOverlay o;
  o.setMap(m);
  o.addScan({{5.05, 4.05, 0.3}}, sensorOver(3.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  const double reach = DynamicOverlayParams{}.block_radius + m->params.clearance_cost_radius;
  double mx, my;
  m->cellXY(cellAt(*m, 5.05, 4.05, 0.0), mx, my);
  size_t blocked = 0, costed = 0;
  for (CellId c = 0; c < m->cells.size(); ++c) {
    const float d = mask->d[c];
    if (std::isinf(d)) {
      continue;
    }
    double x, y;
    m->cellXY(c, x, y);
    EXPECT_NEAR(m->zTop(c), 0.0, 1e-9);                // never the floor above
    EXPECT_NEAR(d, std::hypot(x - mx, y - my), 1e-6);  // the true xy distance to the mark
    EXPECT_LE(d, reach + 1e-6);
    (d <= mask->block_radius ? blocked : costed)++;
  }
  EXPECT_EQ(blocked, 37u);  // the 0.35 m disc at 0.1 m
  EXPECT_GT(costed, blocked);
}

TEST(DynamicOverlay, TheStartDiscIsExemptFromTheDilationButNotFromAMark)
{
  const auto m = build("flat");
  DynamicOverlay o;
  o.setMap(m);
  // A person 0.3 m ahead of a robot standing at (5, 4), seen from behind.
  o.addScan({{5.35, 4.05, 0.3}}, sensorOver(3.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  const CellId start = cellAt(*m, 5.05, 4.05, 0.0), step = cellAt(*m, 5.15, 4.05, 0.0),
               person = cellAt(*m, 5.35, 4.05, 0.0), beyond = cellAt(*m, 5.55, 4.05, 0.0);
  BlockedView plain;
  plain.mask = mask.get();
  const BlockedView view = viewFrom(*mask, start);
  // Without the disc the first step is illegal; with it only the person is.
  EXPECT_TRUE(plain.blocked(*m, step));
  EXPECT_FALSE(view.blocked(*m, step));
  EXPECT_TRUE(plain.blocked(*m, person));
  EXPECT_TRUE(view.blocked(*m, person));
  // Past the person, 0.5 m from the start, the dilation stands.
  EXPECT_TRUE(plain.blocked(*m, beyond));
  EXPECT_TRUE(view.blocked(*m, beyond));
  // The clearance the cost and the shortcut read: none inside the ring,
  // growing beyond it, the static one out of reach.
  EXPECT_FLOAT_EQ(view.clearance(*m, step), 0.0f);
  const CellId near = cellAt(*m, 6.05, 4.05, 0.0);  // 0.7 m from the person
  EXPECT_NEAR(view.clearance(*m, near), 0.7 - mask->block_radius, 1e-6);
  const CellId far = cellAt(*m, 7.05, 4.05, 0.0);  // 1.7 m: out of reach
  EXPECT_FLOAT_EQ(view.clearance(*m, far), m->cells[far].clearance_d);
  // And a search from that start gets out and round.
  SurfaceAStar astar(m);
  SearchParams p;
  const SnapResult g = astar.snap(9.0, 4.0, 0.0, p);
  ASSERT_NE(g.cell, kNoCell);
  const PlanResult r = astar.plan(start, g.cell, p, view);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  for (CellId c : r.cells) {
    EXPECT_FALSE(view.blocked(*m, c));
  }
}

TEST(SurfaceAStar, DetoursRoundAMarkAndPaysForItsBand)
{
  const auto m = build("flat");
  SurfaceAStar astar(m);
  SearchParams p;
  const SnapResult a = astar.snap(2.0, 4.0, 0.0, p), b = astar.snap(10.0, 4.0, 0.0, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r0 = astar.plan(a.cell, b.cell, p);
  ASSERT_EQ(r0.failure, PlanFailure::None);
  // A person across the route at x = 6, seen from 2 m back.
  DynamicOverlay o;
  o.setMap(m);
  std::vector<Waypoint> person;
  for (double y = 3.55; y < 4.6; y += 0.1) {
    person.push_back({6.05, y, 0.3});
  }
  o.addScan(person, sensorOver(4.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  EXPECT_EQ(mask->marked, person.size());
  const BlockedView view = viewFrom(*mask, a.cell);
  const PlanResult r1 = astar.plan(a.cell, b.cell, p, view);
  ASSERT_EQ(r1.failure, PlanFailure::None) << r1.detail;
  for (CellId c : r1.cells) {
    EXPECT_FALSE(view.blocked(*m, c));
  }
  EXPECT_GT(r1.cost, r0.cost);
  // The band beyond the ring costs as a wall's does: a cell 0.5 m from the
  // person pays, the same cell with no person does not.
  const CellId beside = cellAt(*m, 6.55, 4.05, 0.0);
  EXPECT_GT(astar.cellCostFactor(beside, p, &view), astar.cellCostFactor(beside, p));
  // The straight line is in sight without the person and not with them,
  // and the shortcut of the detour keeps off the marks all the way.
  EXPECT_TRUE(lineOfSight(*m, a.cell, b.cell, 0.0f));
  EXPECT_FALSE(lineOfSight(*m, a.cell, b.cell, 0.0f, nullptr, &view));
  const auto chain = shortcutPath(*m, r1.cells, &view);
  EXPECT_LT(chain.size(), r1.cells.size());
  for (size_t i = 1; i < chain.size(); ++i) {
    EXPECT_TRUE(lineOfSight(*m, chain[i - 1], chain[i], 0.0f, nullptr, &view));
  }
  const auto path = buildPath(*m, nullptr, r1.cells, {}, true, 0.1, nullptr, &view);
  ASSERT_GE(path.size(), 2u);
  for (const Waypoint & w : path) {
    const CellId c = m->traversableAt(w.x, w.y, w.z);
    ASSERT_NE(c, kNoCell);
    EXPECT_FALSE(view.blocked(*m, c)) << w.x << ", " << w.y;
  }
}

TEST(SurfaceAStar, AMarkedDoorIsNoPathAndAGoalUnderAPersonDoesNotSnap)
{
  const auto m = build("wall");
  SurfaceAStar astar(m);
  SearchParams p;
  const SnapResult a = astar.snap(3.0, 6.0, 0.0, p), b = astar.snap(9.0, 6.0, 0.0, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  ASSERT_EQ(astar.plan(a.cell, b.cell, p).failure, PlanFailure::None);
  // A person in the door (y 3.6..4.4 at x = 6): the returns on the door's
  // own traversable cells mark, those in the jambs' rings do not, and the
  // dilation closes the whole width.
  DynamicOverlay o;
  o.setMap(m);
  std::vector<Waypoint> door;
  for (double y = 3.65; y < 4.4; y += 0.1) {
    door.push_back({6.05, y, 0.3});
  }
  o.addScan(door, sensorOver(4.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  EXPECT_LT(mask->marked, door.size());
  const BlockedView view = viewFrom(*mask, a.cell);
  const PlanResult r = astar.plan(a.cell, b.cell, p, view);
  // The components are baked, so a closed door is no_path, not
  // disconnected, and the detail says a mark may be why.
  EXPECT_EQ(r.failure, PlanFailure::NoPath);
  EXPECT_NE(r.detail.find("dynamic obstacles"), std::string::npos);
  // A goal under the person does not snap, and the detail counts the cells
  // the marks took; one on the floor before the door still does.
  SearchParams tight = p;
  tight.snap_xy_radius = 0.3;
  const SnapResult under = astar.snap(6.05, 4.05, 0.0, tight, kNoComponent, &view);
  EXPECT_EQ(under.cell, kNoCell);
  EXPECT_NE(under.detail.find("dynamic obstacles"), std::string::npos);
  EXPECT_NE(astar.snap(5.35, 4.05, 0.0, p, kNoComponent, &view).cell, kNoCell);
}

TEST(StairLinks, AMarkOnTheFloorBendsTheStretchAndTheLinkIsStillTaken)
{
  const auto m = build("stairs");
  const auto stairs = [](double x) {
    return x < 4.0 ? 0.0 : 0.17 * std::min(6, static_cast<int>(std::floor((x - 4.0) / 0.29)) + 1);
  };
  const auto links = buildStairLinks(*m, walk(2.0, 10.0, stairs), StairLinkParams{});
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult lo = astar.snap(2.0, 4.0, 0.0, p);
  ASSERT_NE(lo.cell, kNoCell);
  const SnapResult hi = astar.snap(10.0, 4.0, 1.02, p, m->cells[lo.cell].component);
  ASSERT_NE(hi.cell, kNoCell) << hi.detail;
  // A person on the lower floor, on the walked line a metre before the
  // stair, seen from the start.
  DynamicOverlay o;
  o.setMap(m);
  o.addScan(
    {{3.05, 4.05, 0.3}, {3.05, 3.95, 0.3}, {3.05, 4.15, 0.3}}, sensorOver(2.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  const BlockedView view = viewFrom(*mask, lo.cell);
  const PlanResult r = astar.plan(lo.cell, hi.cell, p, view);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  ASSERT_EQ(r.links.size(), 1u);
  for (CellId c : r.cells) {
    EXPECT_FALSE(view.blocked(*m, c));
  }
  const auto path = buildPath(*m, astar.links(), r.cells, r.links, true, 0.1, nullptr, &view);
  ASSERT_GE(path.size(), 2u);
  EXPECT_NEAR(path.back().z, 1.02, 0.25);
  // Every floor pose keeps off the marks; the flight's poses have no
  // traversable cell and are not judged (the climb is blind by decision).
  for (const Waypoint & w : path) {
    const CellId c = m->traversableAt(w.x, w.y, w.z);
    if (c != kNoCell) {
      EXPECT_FALSE(view.blocked(*m, c)) << w.x << ", " << w.y << ", " << w.z;
    }
  }
}

TEST(SurfaceMap, TraversableAtFindsFloorsAndNotFlights)
{
  // What the plugin's isPathValid() puts a path pose on: the traversable
  // surface within three cells of the pose's height, so a pose on a flight
  // finds nothing and the check stops there.
  const auto s = build("stairs");
  EXPECT_NE(s->traversableAt(2.0, 4.0, 0.0), kNoCell);
  EXPECT_NE(s->traversableAt(10.0, 4.0, 1.02), kNoCell);
  EXPECT_EQ(s->traversableAt(5.0, 4.0, 0.68), kNoCell);  // mid-tread, two risers in the disc
  const auto t = build("two_floors");
  EXPECT_NEAR(t->zTop(t->traversableAt(6.0, 4.0, 5.1)), 5.0, 1e-9);
  EXPECT_NEAR(t->zTop(t->traversableAt(6.0, 4.0, -0.1)), 0.0, 1e-9);
  EXPECT_EQ(t->traversableAt(6.0, 4.0, 2.5), kNoCell);  // three cells, not a storey
  EXPECT_EQ(t->traversableAt(-1.0, 4.0, 0.0), kNoCell);
}

// ---- keepout zones (keepout_zones.hpp, 2026-10-23) ----
//
// What a 3D map's forbidden zones rest on: the file contract is enforced, a
// polygon is rasterised on the floor its band names and no other, ringed by
// the robot radius as the 2D KeepoutFilter inflates its mask, the search
// detours round a zone and a zone across the floor is no_path that names the
// zones, the start's ring is exempt but never a core, a zone on a flight cuts
// the stair link, and the two masks combine in one view while clear_overlay
// keeps the zones and a new map drops them.

namespace
{

KeepoutZone rect(
  const std::string & id, double x0, double x1, double y0, double y1, double z_min, double z_max)
{
  KeepoutZone z;
  z.id = id;
  z.polygon = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  z.z_min = z_min;
  z.z_max = z_max;
  return z;
}

BlockedView zoneView(const KeepoutMask & keepout, CellId start)
{
  BlockedView v;
  v.keepout = &keepout;
  v.exempt = start;
  v.exempt_radius = keepout.block_radius;
  return v;
}

}  // namespace

TEST(KeepoutZones, ParsesTheContractAndRefusesWhatBreaksIt)
{
  std::vector<KeepoutZone> zones;
  std::string error;
  const std::string good = R"({ "version": 1, "zones": [
    { "id": "a", "name": "rack", "z_min": 4.5, "z_max": 5.5, "colour": "ignored",
      "points": [{"x": 1, "y": 1}, {"x": 3, "y": 1}, {"x": 3, "y": 2}, {"x": 1, "y": 2},
                 {"x": 1, "y": 1}] },
    { "id": "b", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 0, "y": 1}] },
    { "id": "c", "name": null, "z_min": null, "z_max": 1.0,
      "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 0, "y": 1}] } ] })";
  ASSERT_TRUE(parseKeepoutJson(good, zones, error)) << error;
  ASSERT_EQ(zones.size(), 3u);
  EXPECT_EQ(zones[0].id, "a");
  EXPECT_EQ(zones[0].name, "rack");
  EXPECT_EQ(zones[0].polygon.size(), 4u);  // the closing vertex is dropped
  EXPECT_DOUBLE_EQ(zones[0].z_min, 4.5);
  EXPECT_DOUBLE_EQ(zones[0].z_max, 5.5);
  EXPECT_TRUE(zones[1].name.empty());
  EXPECT_TRUE(std::isinf(zones[1].z_min) && zones[1].z_min < 0);  // absent = unbounded
  EXPECT_TRUE(std::isinf(zones[1].z_max) && zones[1].z_max > 0);
  EXPECT_TRUE(zoneContains(zones[0], 2.0, 1.5, 5.0));
  EXPECT_FALSE(zoneContains(zones[0], 2.0, 1.5, 0.0));  // 1F, under the band
  EXPECT_FALSE(zoneContains(zones[0], 4.0, 1.5, 5.0));  // outside the polygon
  EXPECT_TRUE(zoneContains(zones[1], 0.2, 0.2, -100.0));
  EXPECT_FALSE(zoneContains(zones[1], 0.8, 0.8, 0.0));  // the triangle's far corner
  EXPECT_TRUE(zones[2].name.empty());                   // null = absent
  EXPECT_TRUE(std::isinf(zones[2].z_min));
  EXPECT_DOUBLE_EQ(zones[2].z_max, 1.0);
  // The backend's 2D editor writes exactly this today (id + points, no
  // band): the same file is a 3D map's zones, on every floor.
  const std::string backend_2d =
    R"({"version": 1, "zones": [{"id": "zone-1", "points": [{"x": 0.0, "y": 0.0},)"
    R"( {"x": 2.0, "y": 0.0}, {"x": 2.0, "y": 2.0}, {"x": 0.0, "y": 2.0}]}]})";
  std::vector<KeepoutZone> from_2d;
  ASSERT_TRUE(parseKeepoutJson(backend_2d, from_2d, error)) << error;
  ASSERT_EQ(from_2d.size(), 1u);
  EXPECT_TRUE(zoneContains(from_2d[0], 1.0, 1.0, 0.0));
  EXPECT_TRUE(zoneContains(from_2d[0], 1.0, 1.0, 5.0));
  // Every rule, each refused with the zone named; `zones` stays as it was.
  const char * bad[] = {
    R"({ "zones": [ { "id": "a" } ] })",  // no points
    R"({ "zones": [ { "id": "a", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 1}] } ] })",
    R"({ "zones": [ { "id": "a", "points": [{"x": 0, "y": 0}, {"x": 1, "y": "x"}, {"x": 1, "y": 1}] } ] })",
    R"({ "zones": [ { "id": "a", "points": [[0, 0], [1, 0], [1, 1]] } ] })",  // pairs, not objects
    R"({ "zones": [ { "id": "a", "z_min": 2, "z_max": 1,
                      "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}] } ] })",
    R"({ "zones": [ { "id": "a", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}] },
                    { "id": "a", "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}] } ] })",
    R"({ "zones": [ { "points": [{"x": 0, "y": 0}, {"x": 1, "y": 0}, {"x": 1, "y": 1}] } ] })",
    R"({ "version": 2, "zones": [] })",
    R"([ 1, 2 ])",
    R"({ "zones": 3 })",
    R"(not json)",
  };
  for (const char * text : bad) {
    std::vector<KeepoutZone> out = zones;
    EXPECT_FALSE(parseKeepoutJson(text, out, error)) << text;
    EXPECT_FALSE(error.empty()) << text;
    EXPECT_EQ(out.size(), zones.size()) << text;
  }
  parseKeepoutJson(bad[1], zones, error);
  EXPECT_NE(error.find("'a'"), std::string::npos) << error;
  EXPECT_NE(error.find("points"), std::string::npos) << error;
  // Empty is a valid file: no zones.
  ASSERT_TRUE(parseKeepoutJson(R"({ "zones": [] })", zones, error)) << error;
  EXPECT_TRUE(zones.empty());
}

TEST(KeepoutZones, ARectangleIsRasterisedOnItsFloorOnlyAndRingedByTheRobotRadius)
{
  const auto m = build("two_floors");
  // A 2 x 2 m box on the ground floor: 20 x 20 columns at 0.1 m, every one
  // with a surface at z = 0 and one at z = 5.
  const auto k = buildKeepoutMask(*m, nullptr, {rect("box", 5, 7, 3, 5, -0.5, 0.5)}, 0.25);
  ASSERT_NE(k, nullptr);
  EXPECT_EQ(k->zones, 1u);
  EXPECT_EQ(k->core, 400u);
  EXPECT_DOUBLE_EQ(k->block_radius, 0.25);
  EXPECT_EQ(k->links_cut, 0u);
  size_t core_lo = 0, touched_hi = 0;
  for (CellId c = 0; c < m->cells.size(); ++c) {
    if (std::isinf(k->d[c])) {
      continue;
    }
    if (m->zTop(c) > 2.5) {
      ++touched_hi;
    } else {
      core_lo += k->core_(c);
    }
  }
  EXPECT_EQ(core_lo, 400u);
  EXPECT_EQ(touched_hi, 0u);  // the floor above is never reached
  const CellId inside = cellAt(*m, 6.05, 4.05, 0.0), above = cellAt(*m, 6.05, 4.05, 5.0);
  ASSERT_NE(inside, kNoCell);
  ASSERT_NE(above, kNoCell);
  EXPECT_TRUE(k->core_(inside));
  EXPECT_TRUE(std::isinf(k->d[above]));
  // The ring: distances are between column centres, as the overlay's are.
  // 0.2 m out of the last core column is blocked, 0.3 m is costed, 0.9 m
  // is beyond reach (0.25 + 0.55).
  BlockedView v;
  v.keepout = k.get();
  const CellId r2 = cellAt(*m, 7.15, 4.05, 0.0), r3 = cellAt(*m, 7.25, 4.05, 0.0),
               r9 = cellAt(*m, 7.85, 4.05, 0.0);
  EXPECT_TRUE(v.blocked(*m, r2));
  EXPECT_NEAR(k->d[r2], 0.2, 1e-6);
  EXPECT_FALSE(v.blocked(*m, r3));
  EXPECT_NEAR(k->d[r3], 0.3, 1e-6);
  EXPECT_NEAR(v.clearance(*m, r3), 0.05, 1e-6);
  EXPECT_TRUE(std::isinf(k->d[r9]));
  EXPECT_FLOAT_EQ(v.clearance(*m, r9), m->cells[r9].clearance_d);
  EXPECT_GT(k->blocked, k->core);
  // The same polygon with the upper floor's band forbids 2F and leaves 1F.
  const auto up = buildKeepoutMask(*m, nullptr, {rect("box", 5, 7, 3, 5, 4.5, 5.5)}, 0.25);
  ASSERT_NE(up, nullptr);
  EXPECT_EQ(up->core, 400u);
  EXPECT_TRUE(up->core_(above));
  EXPECT_TRUE(std::isinf(up->d[inside]));
  // A band that holds no floor, or a polygon off the map, is no mask at all.
  EXPECT_EQ(buildKeepoutMask(*m, nullptr, {rect("mid", 5, 7, 3, 5, 2.0, 3.0)}, 0.25), nullptr);
  EXPECT_EQ(buildKeepoutMask(*m, nullptr, {rect("off", 50, 52, 3, 5, -1, 1)}, 0.25), nullptr);
  EXPECT_EQ(buildKeepoutMask(*m, nullptr, {}, 0.25), nullptr);
  // zoneAt finds the zone a core cell is in, for the messages.
  const std::vector<KeepoutZone> zones = {rect("box", 5, 7, 3, 5, -0.5, 0.5)};
  ASSERT_NE(zoneAt(zones, *m, inside), nullptr);
  EXPECT_EQ(zoneAt(zones, *m, inside)->id, "box");
  EXPECT_EQ(zoneAt(zones, *m, above), nullptr);
  EXPECT_EQ(zoneLabel(zones[0]), "'box'");
}

TEST(SurfaceAStar, DetoursRoundAZoneAndAZoneAcrossTheFloorIsNoPathThatSaysSo)
{
  const auto m = build("flat");
  SurfaceAStar astar(m);
  SearchParams p;
  const SnapResult a = astar.snap(2.0, 4.0, 0.0, p), b = astar.snap(10.0, 4.0, 0.0, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r0 = astar.plan(a.cell, b.cell, p);
  ASSERT_EQ(r0.failure, PlanFailure::None);
  // A box across the straight line: the route goes round it, off every
  // blocked cell, and the shortcut and the resampled path stay off too.
  const auto box = buildKeepoutMask(*m, nullptr, {rect("box", 5, 7, 3, 5, -0.5, 0.5)}, 0.25);
  ASSERT_NE(box, nullptr);
  const BlockedView view = zoneView(*box, a.cell);
  const PlanResult r1 = astar.plan(a.cell, b.cell, p, view);
  ASSERT_EQ(r1.failure, PlanFailure::None) << r1.detail;
  EXPECT_GT(r1.cost, r0.cost);
  for (CellId c : r1.cells) {
    EXPECT_FALSE(view.blocked(*m, c));
  }
  const auto path = buildPath(*m, nullptr, r1.cells, {}, true, 0.1, nullptr, &view);
  ASSERT_GE(path.size(), 2u);
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const CellId c = m->traversableAt(path[i].x, path[i].y, path[i].z);
    ASSERT_NE(c, kNoCell);
    EXPECT_FALSE(view.blocked(*m, c)) << path[i].x << ", " << path[i].y;
  }
  EXPECT_GT(len, 8.0);
  // The band beyond the ring costs as a wall's does.
  const CellId beside = cellAt(*m, 7.45, 4.05, 0.0);
  EXPECT_GT(astar.cellCostFactor(beside, p, &view), astar.cellCostFactor(beside, p));
  // A zone across the whole floor: the components are baked, so no_path,
  // and the detail names the zones.
  const auto wall = buildKeepoutMask(*m, nullptr, {rect("wall", 5, 7, 0, 8, -0.5, 0.5)}, 0.25);
  ASSERT_NE(wall, nullptr);
  const BlockedView closed = zoneView(*wall, a.cell);
  const PlanResult r2 = astar.plan(a.cell, b.cell, p, closed);
  EXPECT_EQ(r2.failure, PlanFailure::NoPath);
  EXPECT_NE(r2.detail.find("keepout zones"), std::string::npos) << r2.detail;
  // A goal inside a zone does not snap, and the detail says why; one on the
  // floor before it does.
  const SnapResult in = astar.snap(6.0, 4.0, 0.0, p, kNoComponent, &view);
  EXPECT_EQ(in.cell, kNoCell);
  EXPECT_NE(in.detail.find("keepout zones"), std::string::npos) << in.detail;
  EXPECT_NE(astar.snap(4.0, 4.0, 0.0, p, kNoComponent, &view).cell, kNoCell);
}

TEST(SurfaceAStar, TheStartRingIsExemptButACoreCellIsNot)
{
  const auto m = build("flat");
  SurfaceAStar astar(m);
  SearchParams p;
  const auto box = buildKeepoutMask(*m, nullptr, {rect("box", 5, 7, 3, 5, -0.5, 0.5)}, 0.25);
  ASSERT_NE(box, nullptr);
  const SnapResult g = astar.snap(10.0, 4.0, 0.0, p);
  ASSERT_NE(g.cell, kNoCell);
  // A robot standing 0.2 m outside the zone, inside its ring: its own cell
  // and the ring round it are exempt, so it plans away; a view without the
  // disc would block its first step.
  const CellId edge = cellAt(*m, 7.15, 4.05, 0.0), step = cellAt(*m, 7.25, 4.05, 0.0);
  ASSERT_NE(edge, kNoCell);
  BlockedView plain;
  plain.keepout = box.get();
  EXPECT_TRUE(plain.blocked(*m, edge));
  const BlockedView view = zoneView(*box, edge);
  EXPECT_FALSE(view.blocked(*m, edge));
  EXPECT_FALSE(view.blocked(*m, step));
  const PlanResult r = astar.plan(edge, g.cell, p, view);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  // A core cell is blocked whatever the disc: the plugin refuses the plan
  // before the search (start_in_keepout); the search from deep inside the
  // core finds no way out, since every neighbour is a core cell.
  const CellId inside = cellAt(*m, 6.05, 4.05, 0.0);
  EXPECT_TRUE(box->core_(inside));
  const BlockedView from_inside = zoneView(*box, inside);
  EXPECT_TRUE(from_inside.blocked(*m, cellAt(*m, 6.15, 4.05, 0.0)));
  EXPECT_EQ(astar.plan(inside, g.cell, p, from_inside).failure, PlanFailure::NoPath);
  EXPECT_STREQ(planFailureName(PlanFailure::StartInKeepout), "start_in_keepout");
}

TEST(StairLinks, AZoneOnTheFlightCutsTheLinkAndTheFailureSaysSo)
{
  const auto m = build("stairs");
  const auto stairs = [](double x) {
    return x < 4.0 ? 0.0 : 0.17 * std::min(6, static_cast<int>(std::floor((x - 4.0) / 0.29)) + 1);
  };
  const auto links = buildStairLinks(*m, walk(2.0, 10.0, stairs), StairLinkParams{});
  ASSERT_GE(links->links.size(), 20u);
  SurfaceAStar astar(m, links);
  SearchParams p;
  const SnapResult lo = astar.snap(2.0, 4.0, 0.0, p);
  ASSERT_NE(lo.cell, kNoCell);
  const SnapResult hi = astar.snap(10.0, 4.0, 1.02, p, m->cells[lo.cell].component);
  ASSERT_NE(hi.cell, kNoCell) << hi.detail;
  ASSERT_EQ(astar.plan(lo.cell, hi.cell, p).failure, PlanFailure::None);
  // A zone over the flight's whole width and height: every link has a point
  // in it, so every link is cut and the upper floor is no_path -- not
  // disconnected, the groups are baked -- with the cut named.
  const auto foot = buildKeepoutMask(*m, links.get(), {rect("foot", 4, 6, 0, 8, 0.0, 1.2)}, 0.25);
  ASSERT_NE(foot, nullptr);
  EXPECT_EQ(foot->links_cut, links->links.size());
  for (uint32_t li = 0; li < links->links.size(); ++li) {
    EXPECT_TRUE(foot->linkCut(li));
  }
  const PlanResult cut = astar.plan(lo.cell, hi.cell, p, zoneView(*foot, lo.cell));
  EXPECT_EQ(cut.failure, PlanFailure::NoPath);
  EXPECT_NE(cut.detail.find("stair links"), std::string::npos) << cut.detail;
  // The same polygon with a band above the stair cuts nothing (no link
  // point and no surface in it: no mask), and the link is taken.
  const auto high = buildKeepoutMask(*m, links.get(), {rect("foot", 4, 6, 0, 8, 4.5, 5.5)}, 0.25);
  EXPECT_EQ(high, nullptr);
  // A zone beside the walked line on the lower floor cuts the links whose
  // entries cross it and leaves the rest; the route takes one of those.
  const auto side = buildKeepoutMask(*m, links.get(), {rect("side", 3, 6, 5, 8, -0.5, 1.2)}, 0.25);
  ASSERT_NE(side, nullptr);
  EXPECT_GT(side->links_cut, 0u);
  EXPECT_LT(side->links_cut, links->links.size());
  const PlanResult round = astar.plan(lo.cell, hi.cell, p, zoneView(*side, lo.cell));
  ASSERT_EQ(round.failure, PlanFailure::None) << round.detail;
  ASSERT_EQ(round.links.size(), 1u);
  EXPECT_FALSE(side->linkCut(round.links[0].link));
}

TEST(BlockedView, AMarkAndAZoneCombineAsOrAndTheirClearanceAsMin)
{
  const auto m = build("flat");
  DynamicOverlay o;
  o.setMap(m);
  // A person at x = 4 and a zone at x = 6..8, both across y = 4.
  o.addScan({{4.05, 4.05, 0.3}}, sensorOver(2.05, 4.05, 0.0), 0.0);
  const auto mask = o.snapshot(0.0);
  ASSERT_NE(mask, nullptr);
  const auto zone = buildKeepoutMask(*m, nullptr, {rect("z", 6, 8, 3, 5, -0.5, 0.5)}, 0.25);
  ASSERT_NE(zone, nullptr);
  BlockedView both;
  both.mask = mask.get();
  both.keepout = zone.get();
  EXPECT_FALSE(both.empty());
  EXPECT_TRUE(BlockedView{}.empty());
  const CellId person = cellAt(*m, 4.05, 4.05, 0.0), in_zone = cellAt(*m, 7.05, 4.05, 0.0),
               between = cellAt(*m, 5.05, 4.05, 0.0), near_zone = cellAt(*m, 5.75, 4.05, 0.0),
               near_person = cellAt(*m, 4.55, 4.05, 0.0);
  EXPECT_TRUE(both.blocked(*m, person));
  EXPECT_TRUE(both.blockedByMark(*m, person));
  EXPECT_FALSE(both.blockedByZone(*m, person));
  EXPECT_TRUE(both.blocked(*m, in_zone));
  EXPECT_TRUE(both.blockedByZone(*m, in_zone));
  EXPECT_FALSE(both.blockedByMark(*m, in_zone));
  EXPECT_FALSE(both.blocked(*m, between));
  // 0.5 m from the person (ring 0.35): 0.15 beyond it; 0.3 m from the zone's
  // first core column (ring 0.25): 0.05 beyond it. Each cell's clearance is
  // the smaller of its two distances-beyond and the static one.
  EXPECT_NEAR(both.clearance(*m, near_person), 0.5 - mask->block_radius, 1e-6);
  EXPECT_NEAR(both.clearance(*m, near_zone), 0.3 - zone->block_radius, 1e-6);
  BlockedView zone_only;
  zone_only.keepout = zone.get();
  EXPECT_FLOAT_EQ(zone_only.clearance(*m, near_person), m->cells[near_person].clearance_d);
  // The overlay carries the keepout for the validity check: clear() (what
  // clear_overlay calls) keeps it, setMap() (a reload) drops it, and
  // setKeepout() on no map stores nothing.
  o.setKeepout(zone);
  EXPECT_EQ(o.mapAndKeepout().second, zone);
  EXPECT_EQ(o.clear(), 1u);
  EXPECT_EQ(o.mapAndKeepout().second, zone);
  EXPECT_EQ(o.snapshot(0.0), nullptr);
  o.setMap(build("two_floors"));
  EXPECT_EQ(o.mapAndKeepout().second, nullptr);
  o.setMap(nullptr);
  o.setKeepout(zone);
  EXPECT_EQ(o.mapAndKeepout().second, nullptr);
}

// ---- Kerbs (2026-10-29) -------------------------------------------------
//
// The 0.15 m step of the kerb scene is one piece of ground (above), and
// since 2026-10-29 also a kerb: its cells are step edges with a cost band,
// its low side is one kerb line, kerb links cross it square, and a route
// takes it on a link, never along it or diagonally. The ramp scene has
// one-cell steps everywhere (a 15 deg slope quantised to 0.1 m voxels) and
// no kerb; the stairs scene's first riser is not one either.

namespace
{

// The heading (deg) of the path's segment through x.
double headingAtX(const std::vector<Waypoint> & path, double x)
{
  for (size_t i = 1; i < path.size(); ++i) {
    if ((path[i - 1].x - x) * (path[i].x - x) <= 0 && path[i].x != path[i - 1].x) {
      return std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x) * 180.0 / M_PI;
    }
  }
  return 1e9;
}

}  // namespace

TEST(Kerbs, StepEdgesHaveABandAndOnlyTheKerbIsALine)
{
  BuildStats s;
  const auto m = build("kerb", &s);
  EXPECT_EQ(s.kerb_lines, 1u);
  EXPECT_GE(s.kerb_cells, 70u);  // the 8 m line, less the footprint rule's rim
  ASSERT_EQ(m->kerb_lines.size(), 1u);
  // The low side at x = 5.95 is the line, flagged; its band reaches 0.4 m
  // either side with the kerb's rise, and no further.
  const CellId edge = cellAt(*m, 5.95, 4.0, 0.0);
  ASSERT_NE(edge, kNoCell);
  EXPECT_TRUE(m->cells[edge].flags & kKerb);
  EXPECT_EQ(m->cells[edge].step_rise, 2);
  EXPECT_EQ(m->cells[edge].edge_rise, 2);
  EXPECT_NEAR(m->cells[edge].step_d, 0.0, 1e-6);
  const CellId near = cellAt(*m, 6.25, 4.0, 0.15);
  ASSERT_NE(near, kNoCell);
  EXPECT_FALSE(m->cells[near].flags & kKerb);
  EXPECT_EQ(m->cells[near].edge_rise, 2);
  EXPECT_NEAR(m->cells[near].step_d, 0.2, 1e-6);
  const CellId far = cellAt(*m, 3.0, 4.0, 0.0);
  ASSERT_NE(far, kNoCell);
  EXPECT_EQ(m->cells[far].edge_rise, 0);
  EXPECT_NEAR(m->cells[far].step_d, m->params.step_edge_cost_radius, 1e-6);
  // The ramp: one-cell steps all over its slope, none sure, no kerb; the
  // stairs: the floor's edge along the first riser is not a kerb either,
  // the next riser being 0.3 m on.
  BuildStats r;
  build("ramp", &r);
  EXPECT_GT(r.step_edges, 0u);
  EXPECT_EQ(r.sure_step_edges, 0u);
  EXPECT_EQ(r.kerb_lines, 0u);
  BuildStats st;
  build("stairs", &st);
  EXPECT_EQ(st.kerb_lines, 0u);
}

TEST(Kerbs, KerbLinksCrossSquareWithALeadEitherSide)
{
  const auto m = build("kerb");
  const auto links = buildStairLinks(*m, {}, StairLinkParams{});
  ASSERT_GE(links->links.size(), 60u);
  EXPECT_EQ(links->kerbs, 1u);
  EXPECT_TRUE(links->notes.empty());
  for (const StairLink & l : links->links) {
    EXPECT_TRUE(l.kerb && l.straight && l.detected);
    EXPECT_NEAR(std::fabs(l.up_x), 1.0, 0.05);  // square to the kerb along y
    ASSERT_EQ(l.pts.size(), 4u);
    EXPECT_NEAR(l.pts.front().x, 5.5, 0.11);  // half a metre before the edge ...
    EXPECT_NEAR(l.pts.back().x, 6.5, 0.11);   // ... and past it
    EXPECT_NEAR(l.pts[1].z, 0.0, 1e-6);
    EXPECT_NEAR(l.pts[2].z, 0.2, 1e-6);                           // the 0.15 m step's voxel top
    EXPECT_EQ(m->cells[l.a].component, m->cells[l.b].component);  // joins nothing
  }
  // The cells the links cross are marked: a step between them is a crossing.
  const CellId lo = cellAt(*m, 5.95, 4.0, 0.0), hi = cellAt(*m, 6.05, 4.0, 0.15);
  ASSERT_NE(lo, kNoCell);
  ASSERT_NE(hi, kNoCell);
  EXPECT_TRUE(links->kerbCrossing(*m, lo, hi));
  EXPECT_FALSE(links->kerbCrossing(*m, lo, cellAt(*m, 5.95, 4.1, 0.0)));  // along the edge
}

TEST(Kerbs, ARouteTakesTheKerbOnALinkAndNeverAlongIt)
{
  const auto m = build("kerb");
  const auto links = buildStairLinks(*m, {}, StairLinkParams{});
  SurfaceAStar astar(m, links);
  SearchParams p;
  // Diagonally across: the route meets the kerb square and the shortcut
  // does not put the diagonal back.
  const SnapResult a = astar.snap(2.0, 2.0, 0.0, p);
  const SnapResult b = astar.snap(10.0, 6.0, 0.15, p);
  ASSERT_NE(a.cell, kNoCell);
  ASSERT_NE(b.cell, kNoCell);
  const PlanResult r = astar.plan(a.cell, b.cell, p);
  ASSERT_EQ(r.failure, PlanFailure::None) << r.detail;
  ASSERT_EQ(r.links.size(), 1u);
  EXPECT_TRUE(links->links[r.links[0].link].kerb);
  const auto path = buildPath(*m, astar.links(), r.cells, r.links, true, 0.1);
  EXPECT_NEAR(headingAtX(path, 6.0), 0.0, 5.0);
  EXPECT_NEAR(headingAtX(path, 5.7), 0.0, 5.0);  // still square half a metre before ...
  EXPECT_NEAR(headingAtX(path, 6.3), 0.0, 5.0);  // ... and after
  for (size_t i = 1; i < path.size(); ++i) {
    if (path[i - 1].x > 5.52 && path[i].x < 6.48) {  // on the link: one straight line
      EXPECT_NEAR(path[i].y, path[i - 1].y, 1e-6);
    }
  }
  // Along the kerb, a hand's breadth from it: the band pushes the route out
  // to the band's edge and it never steps up.
  const SnapResult c = astar.snap(5.85, 1.0, 0.0, p);
  const SnapResult d = astar.snap(5.85, 7.0, 0.0, p);
  ASSERT_NE(c.cell, kNoCell);
  ASSERT_NE(d.cell, kNoCell);
  const PlanResult along = astar.plan(c.cell, d.cell, p);
  ASSERT_EQ(along.failure, PlanFailure::None) << along.detail;
  EXPECT_TRUE(along.links.empty());
  double xmin = 1e9;
  for (const CellId cell : along.cells) {
    double x, y;
    m->cellXY(cell, x, y);
    xmin = std::min(xmin, x);
    EXPECT_LT(x, 6.0);
  }
  EXPECT_LT(xmin, 5.6);
  // With the links off the old behaviour: the diagonal route steps over the
  // kerb wherever it likes.
  StairLinkParams off;
  off.kerb_links = false;
  SurfaceAStar plain(m, buildStairLinks(*m, {}, off));
  const PlanResult old = plain.plan(a.cell, b.cell, p);
  ASSERT_EQ(old.failure, PlanFailure::None) << old.detail;
  EXPECT_TRUE(old.links.empty());
}
