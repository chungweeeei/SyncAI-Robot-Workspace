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
    case PlanFailure::StartInKeepout:
      return "start_in_keepout";
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

SurfaceAStar::SurfaceAStar(
  std::shared_ptr<const SurfaceMap> map, std::shared_ptr<const StairLinks> links)
: map_(std::move(map)),
  links_(std::move(links)),
  g_(map_->cells.size(), 0.f),
  parent_(map_->cells.size(), kNoCell),
  parent_link_(map_->cells.size(), -1),
  open_stamp_(map_->cells.size(), 0),
  closed_stamp_(map_->cells.size(), 0)
{
}

uint32_t SurfaceAStar::reach(CellId c) const
{
  const uint32_t comp = map_->cells[c].component;
  return links_ ? links_->groupOf(comp) : comp;
}

double SurfaceAStar::cellCostFactor(
  CellId c, const SearchParams & p, const BlockedView * blocked) const
{
  const SurfaceCell & cell = map_->cells[c];
  double f = 1.0;
  // The marks count as boundary: the clearance is the smaller of the static
  // one and the distance beyond the nearest mark's blocked ring, so the same
  // curve keeps the path off a person that keeps it off a wall.
  const float clearance = blocked ? blocked->clearance(*map_, c) : cell.clearance_d;
  if (clearance < map_->params.clearance_cost_radius) {
    f += p.clearance_cost_weight * std::exp(-p.clearance_cost_scaling * clearance);
  }
  if (cell.flags & kInferred) {
    f *= p.inferred_cost_multiplier;
  }
  if (cell.gap_cnt > 0 && map_->footprint_n > 0) {
    f *= 1.0 + p.gap_cost_weight * cell.gap_cnt / static_cast<double>(map_->footprint_n);
  }
  return f;
}

SnapResult SurfaceAStar::snap(
  double x, double y, double z_hint, const SearchParams & p, uint32_t prefer_component,
  const BlockedView * blocked) const
{
  const SurfaceMap & m = *map_;
  SnapResult out;
  // Traversable cells within the radius the view refused, by source.
  size_t n_marked = 0, n_zoned = 0;
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
  const uint32_t want = links_ && prefer_component != kNoComponent
                          ? links_->groupOf(prefer_component)
                          : prefer_component;
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
        if (blocked && blocked->blocked(m, i)) {
          // A cell both refuse counts as a zone's: the zone is permanent and
          // the one the operator can do something about.
          (blocked->blockedByZone(m, i) ? n_zoned : n_marked)++;
          continue;
        }
        if (prefer_component != kNoComponent && reach(i) != want) {
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
  if (n_marked > 0) {
    out.detail += "; " + std::to_string(n_marked) +
                  " traversable cells within the radius are blocked by dynamic obstacles";
  }
  if (n_zoned > 0) {
    out.detail +=
      "; " + std::to_string(n_zoned) + " traversable cells within the radius are in keepout zones";
  }
  return out;
}

PlanResult SurfaceAStar::plan(
  CellId start, CellId goal, const SearchParams & p, const BlockedView & blocked)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  const SurfaceMap & m = *map_;
  PlanResult r;
  if (start == goal) {
    r.cells.push_back(start);
    return r;
  }
  if (reach(start) != reach(goal)) {
    r.failure = PlanFailure::Disconnected;
    char buf[160];
    std::snprintf(
      buf, sizeof buf,
      "start (component %u, z %.2f) and goal (component %u, z %.2f) are on disconnected ground "
      "(no stair link joins them)",
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
  parent_link_[start] = -1;
  const double stair_factor = std::max(1.0, p.stair_cost_multiplier);
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
      std::vector<int32_t> via;
      for (CellId k = goal; k != kNoCell; k = parent_[k]) {
        r.cells.push_back(k);
        via.push_back(parent_link_[k]);
      }
      std::reverse(r.cells.begin(), r.cells.end());
      std::reverse(via.begin(), via.end());
      // via[i] is the link that led INTO cells[i], i.e. the move i-1 -> i.
      for (size_t i = 1; i < r.cells.size(); ++i) {
        if (via[i] >= 0) {
          const StairLink & l = links_->links[static_cast<size_t>(via[i])];
          r.links.push_back({i - 1, static_cast<uint32_t>(via[i]), l.a != r.cells[i - 1]});
        }
      }
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
      if (closed_stamp_[n] == gen || blocked.blocked(m, n)) {
        return;
      }
      const double dz = m.zTop(n) - zc;
      const double len = std::sqrt(
        (SurfaceMap::kDx8[k] * SurfaceMap::kDx8[k] + SurfaceMap::kDy8[k] * SurfaceMap::kDy8[k]) *
          m.res * m.res +
        dz * dz);
      const float ng = g_[c] + static_cast<float>(len * cellCostFactor(n, p, &blocked));
      if (open_stamp_[n] != gen || ng < g_[n]) {
        open_stamp_[n] = gen;
        g_[n] = ng;
        parent_[n] = c;
        parent_link_[n] = -1;
        open.push({ng + static_cast<float>(heuristic(n)), n});
      }
    });
    if (links_) {
      const auto it = links_->at.find(c);
      if (it != links_->at.end()) {
        for (const uint32_t li : it->second) {
          // A link the keepout zones cut (a point of it in a core) is not a
          // move: the one way a zone forbids a stair.
          if (blocked.keepout && blocked.keepout->linkCut(li)) {
            continue;
          }
          const StairLink & l = links_->links[li];
          const CellId n = l.a == c ? l.b : l.a;
          if (closed_stamp_[n] == gen) {
            continue;
          }
          const float ng = g_[c] + static_cast<float>(l.length * stair_factor);
          if (open_stamp_[n] != gen || ng < g_[n]) {
            open_stamp_[n] = gen;
            g_[n] = ng;
            parent_[n] = c;
            parent_link_[n] = static_cast<int32_t>(li);
            open.push({ng + static_cast<float>(heuristic(n)), n});
          }
        }
      }
    }
  }
  if (r.failure == PlanFailure::None) {
    r.failure = PlanFailure::NoPath;
    r.detail = "the open set ran out before the goal was reached";
    // The components and the linked groups are baked, so this is the one
    // way a mark or a zone shows up as a failure: say so, or a route that
    // existed yesterday "has no path".
    if (blocked.mask) {
      r.detail += " (" + std::to_string(blocked.mask->marked) +
                  " cells marked by dynamic obstacles; a mark may have closed the way)";
    }
    if (blocked.keepout) {
      r.detail += " (" + std::to_string(blocked.keepout->zones) + " keepout zones block " +
                  std::to_string(blocked.keepout->blocked) + " cells and cut " +
                  std::to_string(blocked.keepout->links_cut) +
                  " stair links; a zone may have closed the way)";
    }
  }
  r.search_s = std::chrono::duration<double>(Clock::now() - t0).count();
  return r;
}

}  // namespace syncai_global_planner::octomap_planner_3d
