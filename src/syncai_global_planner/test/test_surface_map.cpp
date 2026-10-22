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

#include "syncai_global_planner/plugins/octomap_planner_3d/dynamic_overlay.hpp"
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
