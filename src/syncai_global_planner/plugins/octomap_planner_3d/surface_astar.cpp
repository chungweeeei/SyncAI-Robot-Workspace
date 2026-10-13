#include "syncai_global_planner/plugins/octomap_planner_3d/surface_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <queue>
#include <sstream>

namespace syncai_global_planner::octomap_planner_3d
{

const char * planFailureName(PlanFailure f)
{
  switch (f) {
    case PlanFailure::None:
      return "none";
    case PlanFailure::MapNotLoaded:
      return "map_not_loaded";
    case PlanFailure::MapLoading:
      return "map_loading";
    case PlanFailure::StartOutsideMap:
      return "start_outside_map";
    case PlanFailure::GoalOutsideMap:
      return "goal_outside_map";
    case PlanFailure::StartUnavailable:
      return "start_unavailable";
    case PlanFailure::GoalUnavailable:
      return "goal_unavailable";
    case PlanFailure::Disconnected:
      return "disconnected";
    case PlanFailure::SearchLimit:
      return "search_limit";
    case PlanFailure::Timeout:
      return "timeout";
    case PlanFailure::NoPath:
      return "no_path";
    default:
      return "?";
  }
}

SurfaceAStar::SurfaceAStar(std::shared_ptr<const SurfaceMap> map)
: map_(std::move(map)),
  g_(map_->cells.size(), 0.f),
  parent_(map_->cells.size(), kNoCell),
  open_stamp_(map_->cells.size(), 0),
  closed_stamp_(map_->cells.size(), 0)
{
}

double SurfaceAStar::cellCostFactor(CellId c, const SearchParams & p) const
{
  const SurfaceCell & cell = map_->cells[c];
  double f = 1.0;
  if (cell.clearance_d < map_->params.clearance_cost_radius) {
    f += p.clearance_cost_weight * std::exp(-p.clearance_cost_scaling * cell.clearance_d);
  }
  if (cell.flags & kInferred) {
    f *= p.inferred_cost_multiplier;
  }
  return f;
}

SnapResult SurfaceAStar::snap(
  double x, double y, double z_hint, const SearchParams & p, uint32_t prefer_component) const
{
  const SurfaceMap & m = *map_;
  SnapResult out;
  const int cx = m.ixOf(x), cy = m.iyOf(y);
  if (!m.inGrid(cx, cy)) {
    out.outside = true;
    char buf[160];
    std::snprintf(
      buf, sizeof buf, "(%.2f, %.2f) is outside the map (x %.1f..%.1f, y %.1f..%.1f)", x, y, m.x0,
      m.x0 + m.nx * m.res, m.y0, m.y0 + m.ny * m.res);
    out.detail = buf;
    return out;
  }
  const int r = static_cast<int>(std::ceil(p.snap_xy_radius / m.res));
  CellId best = kNoCell, best_other = kNoCell;
  double best_xy = 0, best_dz = 0, other_xy = 0;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const int ix = cx + dx, iy = cy + dy;
      if (!m.inGrid(ix, iy)) {
        continue;
      }
      const double xy = std::hypot(m.cx(ix) - x, m.cy(iy) - y);
      if (xy > p.snap_xy_radius) {
        continue;
      }
      const uint32_t c = m.col(ix, iy);
      for (uint32_t i = m.col_begin[c], e = m.col_begin[c + 1]; i < e; ++i) {
        if (!m.traversable(i)) {
          continue;
        }
        const double dz = m.zTop(i) - z_hint;
        if (std::fabs(dz) > p.snap_z_window) {
          continue;
        }
        if (prefer_component != kNoComponent && m.cells[i].component != prefer_component) {
          if (best_other == kNoCell || xy < other_xy) {
            best_other = i;
            other_xy = xy;
          }
          continue;
        }
        if (
          best == kNoCell || xy < best_xy - 1e-9 ||
          (std::fabs(xy - best_xy) <= 1e-9 && std::fabs(dz) < std::fabs(best_dz))) {
          best = i;
          best_xy = xy;
          best_dz = dz;
        }
      }
    }
  }
  if (best != kNoCell) {
    out.cell = best;
    out.xy_dist = best_xy;
    out.dz = best_dz;
    return out;
  }
  char buf[256];
  if (best_other != kNoCell) {
    double ox, oy;
    m.cellXY(best_other, ox, oy);
    std::snprintf(
      buf, sizeof buf,
      "the nearest traversable surface to (%.2f, %.2f, z %.2f) is (%.2f, %.2f, %.2f) in component "
      "%u, "
      "not reachable from the start (component %u)",
      x, y, z_hint, ox, oy, m.zTop(best_other), m.cells[best_other].component, prefer_component);
  } else {
    std::snprintf(
      buf, sizeof buf,
      "no traversable surface within %.2f m xy / +-%.2f m z of (%.2f, %.2f, z %.2f); surfaces in "
      "that "
      "column: ",
      p.snap_xy_radius, p.snap_z_window, x, y, z_hint);
  }
  out.detail = buf;
  if (best_other == kNoCell) {
    out.detail += m.describeColumn(m.col(cx, cy));
  }
  return out;
}

PlanResult SurfaceAStar::plan(CellId start, CellId goal, const SearchParams & p)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const SurfaceMap & m = *map_;
  PlanResult r;
  if (start == goal) {
    r.cells.push_back(start);
    return r;
  }
  if (m.cells[start].component != m.cells[goal].component) {
    r.failure = PlanFailure::Disconnected;
    char buf[160];
    std::snprintf(
      buf, sizeof buf,
      "start (component %u, z %.2f) and goal (component %u, z %.2f) are on disconnected ground",
      m.cells[start].component, m.zTop(start), m.cells[goal].component, m.zTop(goal));
    r.detail = buf;
    return r;
  }

  if (++gen_ == 0) {
    std::fill(open_stamp_.begin(), open_stamp_.end(), 0u);
    std::fill(closed_stamp_.begin(), closed_stamp_.end(), 0u);
    gen_ = 1;
  }
  const uint32_t gen = gen_;
  double gx, gy;
  m.cellXY(goal, gx, gy);
  const double gz = m.zTop(goal);
  auto heuristic = [&](CellId c) {
    double x, y;
    m.cellXY(c, x, y);
    return p.heuristic_weight *
           std::sqrt(
             (x - gx) * (x - gx) + (y - gy) * (y - gy) + (m.zTop(c) - gz) * (m.zTop(c) - gz));
  };

  using Item = std::pair<float, CellId>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  g_[start] = 0.f;
  parent_[start] = kNoCell;
  open_stamp_[start] = gen;
  open.push({static_cast<float>(heuristic(start)), start});

  while (!open.empty()) {
    const CellId c = open.top().second;
    open.pop();
    if (closed_stamp_[c] == gen) {
      continue;  // a stale heap entry
    }
    closed_stamp_[c] = gen;
    ++r.iterations;
    if (c == goal) {
      for (CellId k = goal; k != kNoCell; k = parent_[k]) {
        r.cells.push_back(k);
      }
      std::reverse(r.cells.begin(), r.cells.end());
      r.cost = g_[goal];
      r.search_s = std::chrono::duration<double>(Clock::now() - t0).count();
      return r;
    }
    if (r.iterations >= p.max_iterations) {
      r.failure = PlanFailure::SearchLimit;
      r.detail = "max_iterations (" + std::to_string(p.max_iterations) + ") reached";
      break;
    }
    if (p.max_planning_time > 0 && (r.iterations & 1023) == 0) {
      r.search_s = std::chrono::duration<double>(Clock::now() - t0).count();
      if (r.search_s > p.max_planning_time) {
        r.failure = PlanFailure::Timeout;
        char buf[96];
        std::snprintf(
          buf, sizeof buf, "max_planning_time (%.2f s) exceeded after %zu iterations",
          p.max_planning_time, r.iterations);
        r.detail = buf;
        break;
      }
    }
    const double zc = m.zTop(c);
    m.forEachMove(c, [&](CellId n, int k) {
      if (closed_stamp_[n] == gen) {
        return;
      }
      const double dz = m.zTop(n) - zc;
      const double len = std::sqrt(
        (SurfaceMap::kDx8[k] * SurfaceMap::kDx8[k] + SurfaceMap::kDy8[k] * SurfaceMap::kDy8[k]) *
          m.res * m.res +
        dz * dz);
      const float ng = g_[c] + static_cast<float>(len * cellCostFactor(n, p));
      if (open_stamp_[n] != gen || ng < g_[n]) {
        open_stamp_[n] = gen;
        g_[n] = ng;
        parent_[n] = c;
        open.push({ng + static_cast<float>(heuristic(n)), n});
      }
    });
  }
  if (r.failure == PlanFailure::None) {
    r.failure = PlanFailure::NoPath;
    r.detail = "the open set ran out before the goal was reached";
  }
  r.search_s = std::chrono::duration<double>(Clock::now() - t0).count();
  return r;
}

}  // namespace syncai_global_planner::octomap_planner_3d
