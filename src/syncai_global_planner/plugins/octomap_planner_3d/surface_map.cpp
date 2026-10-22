#include "syncai_global_planner/plugins/octomap_planner_3d/surface_map.hpp"

#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <queue>
#include <sstream>
#include <utility>

namespace syncai_global_planner::octomap_planner_3d
{

const char * rejectName(Reject r)
{
  switch (r) {
    case Reject::None:
      return "none";
    case Reject::ClearanceOccupied:
      return "clearance_occupied";
    case Reject::ClearanceUnknown:
      return "clearance_unknown";
    case Reject::NoFreeAbove:
      return "no_free_above";
    case Reject::Step:
      return "step";
    case Reject::FootprintBlocked:
      return "footprint_blocked";
    case Reject::FootprintGap:
      return "footprint_gap";
    case Reject::Slope:
      return "slope";
    case Reject::Residual:
      return "residual";
    default:
      return "?";
  }
}

bool loadOctomapBt(const std::string & path, octomap::OcTree & tree, std::string & error)
{
  // readBinary clears the tree, takes the resolution from the file header and
  // returns false (after an OCTOMAP_ERROR on stderr) on a missing file or a
  // header that is not an OcTree's. It does not throw.
  if (!tree.readBinary(path)) {
    error = "cannot read '" + path + "' as an OcTree binary (.bt) file";
    return false;
  }
  if (tree.size() == 0) {
    error = "'" + path + "' holds an empty tree";
    return false;
  }
  return true;
}

CellId SurfaceMap::layerNear(uint32_t c, int kz, int max_dkz) const
{
  CellId best = kNoCell;
  int best_d = max_dkz + 1;
  for (uint32_t i = col_begin[c], e = col_begin[c + 1]; i < e; ++i) {
    const int d = std::abs(static_cast<int>(cells[i].kz) - kz);
    if (d < best_d) {
      best_d = d;
      best = i;
    } else if (cells[i].kz > kz) {
      break;  // sorted by kz: nothing closer follows
    }
  }
  return best;
}

CellId SurfaceMap::traversableNeighbour(int ix, int iy, int kz) const
{
  if (!inGrid(ix, iy)) {
    return kNoCell;
  }
  const CellId n = layerNear(col(ix, iy), kz, step_cells);
  return (n != kNoCell && traversable(n)) ? n : kNoCell;
}

std::string SurfaceMap::describeColumn(uint32_t c) const
{
  if (c >= columns() || col_begin[c] == col_begin[c + 1]) {
    return "no surface";
  }
  std::ostringstream os;
  os << "z=[";
  for (uint32_t i = col_begin[c], e = col_begin[c + 1]; i < e; ++i) {
    if (i != col_begin[c]) {
      os << ", ";
    }
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.2f", zTop(i));
    os << buf << " (";
    if (traversable(i)) {
      os << "traversable, component " << cells[i].component;
    } else {
      os << "untraversable: " << rejectName(static_cast<Reject>(cells[i].reject));
    }
    if (inferred(i)) {
      os << ", inferred";
    }
    os << ")";
  }
  os << "]";
  return os.str();
}

std::vector<Offset> discOffsets(double radius_m, double res, bool include_centre)
{
  std::vector<Offset> out;
  const int r = static_cast<int>(std::ceil(radius_m / res));
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx == 0 && dy == 0 && !include_centre) {
        continue;
      }
      const double d = std::hypot(dx, dy) * res;
      if (d > radius_m + 1e-9) {
        continue;
      }
      int q;
      if (dx >= 0 && dy > 0) {
        q = 0;
      } else if (dx < 0 && dy >= 0) {
        q = 1;
      } else if (dx <= 0 && dy < 0) {
        q = 2;
      } else {
        q = 3;  // dx > 0 && dy <= 0, and the centre
      }
      out.push_back({dx, dy, static_cast<float>(d), q});
    }
  }
  std::sort(
    out.begin(), out.end(), [](const Offset & a, const Offset & b) { return a.d_m < b.d_m; });
  return out;
}

namespace
{

using Clock = std::chrono::steady_clock;

double since(Clock::time_point t0)
{
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

// One bit per voxel of a column, for "known" (a leaf covers it) and
// "occupied". Built in one pass over the leaves so that no stage below ever
// calls OcTree::search(): 1.25 M columns x a few lookups each would be the
// slowest part of the build by far, and search() on a pruned tree returns the
// block for any key inside it, which the masks reproduce exactly.
struct ColumnMasks
{
  int nz, words;
  std::vector<uint64_t> known, occ;

  ColumnMasks(size_t ncols, int nz_)
  : nz(nz_), words((nz_ + 63) / 64), known(ncols * words, 0), occ(ncols * words, 0)
  {
  }

  const uint64_t * k(size_t c) const { return &known[c * words]; }
  const uint64_t * o(size_t c) const { return &occ[c * words]; }
  uint64_t * k(size_t c) { return &known[c * words]; }
  uint64_t * o(size_t c) { return &occ[c * words]; }

  // The mask of word b's bits that fall in [zlo, zhi).
  static uint64_t wordMask(int b, int zlo, int zhi)
  {
    const int lo = std::max(zlo - (b << 6), 0);
    const int hi = std::min(zhi - (b << 6), 64);
    const uint64_t upper = (hi >= 64) ? ~0ULL : ((1ULL << hi) - 1);
    return upper & ~((1ULL << lo) - 1);
  }
  static void setRange(uint64_t * w, int zlo, int zhi)
  {
    for (int b = zlo >> 6, be = (zhi - 1) >> 6; b <= be; ++b) {
      w[b] |= wordMask(b, zlo, zhi);
    }
  }
  static bool anyInRange(const uint64_t * w, int zlo, int zhi)
  {
    for (int b = zlo >> 6, be = (zhi - 1) >> 6; b <= be; ++b) {
      if (w[b] & wordMask(b, zlo, zhi)) {
        return true;
      }
    }
    return false;
  }
  static int popcountRange(const uint64_t * w, int zlo, int zhi)
  {
    int n = 0;
    for (int b = zlo >> 6, be = (zhi - 1) >> 6; b <= be; ++b) {
      n += __builtin_popcountll(w[b] & wordMask(b, zlo, zhi));
    }
    return n;
  }
};

// Re-sort cells by (column, kz) and recompute col_begin. Needed after the
// hole-fill appends cells out of order.
void rebuildCsr(SurfaceMap & m)
{
  std::vector<uint32_t> idx(m.cells.size());
  std::iota(idx.begin(), idx.end(), 0u);
  std::sort(idx.begin(), idx.end(), [&m](uint32_t a, uint32_t b) {
    if (m.cell_col[a] != m.cell_col[b]) {
      return m.cell_col[a] < m.cell_col[b];
    }
    return m.cells[a].kz < m.cells[b].kz;
  });
  std::vector<SurfaceCell> cells(m.cells.size());
  std::vector<uint32_t> cols(m.cells.size());
  for (size_t i = 0; i < idx.size(); ++i) {
    cells[i] = m.cells[idx[i]];
    cols[i] = m.cell_col[idx[i]];
  }
  m.cells.swap(cells);
  m.cell_col.swap(cols);
  m.col_begin.assign(m.columns() + 1, 0);
  for (uint32_t c : m.cell_col) {
    ++m.col_begin[c + 1];
  }
  for (size_t c = 0; c < m.columns(); ++c) {
    m.col_begin[c + 1] += m.col_begin[c];
  }
}

// ---- stage 1: leaves -> column masks --------------------------------------

void fillColumnMasks(
  const octomap::OcTree & tree, const SurfaceMap & m, const octomap::OcTreeKey & k0,
  ColumnMasks & masks, BuildStats & stats)
{
  const unsigned tree_depth = tree.getTreeDepth();
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    ++stats.leaves;
    const bool occupied = tree.isNodeOccupied(*it);
    (occupied ? stats.occupied_leaves : stats.free_leaves)++;
    // A pruned leaf at depth d covers 2^(tree_depth - d) cells per axis; its
    // index key is the lowest key of the block (OcTreeKey.h computeIndexKey).
    const unsigned level = tree_depth - it.getDepth();
    const int n = 1 << level;
    const octomap::OcTreeKey kmin = it.getIndexKey();
    const int xlo = std::max(static_cast<int>(kmin[0]) - static_cast<int>(k0[0]), 0);
    const int xhi = std::min(static_cast<int>(kmin[0]) + n - static_cast<int>(k0[0]), m.nx);
    const int ylo = std::max(static_cast<int>(kmin[1]) - static_cast<int>(k0[1]), 0);
    const int yhi = std::min(static_cast<int>(kmin[1]) + n - static_cast<int>(k0[1]), m.ny);
    const int zlo = std::max(static_cast<int>(kmin[2]) - static_cast<int>(k0[2]), 0);
    const int zhi = std::min(static_cast<int>(kmin[2]) + n - static_cast<int>(k0[2]), m.nz);
    if (xlo >= xhi || ylo >= yhi || zlo >= zhi) {
      continue;
    }
    for (int iy = ylo; iy < yhi; ++iy) {
      for (int ix = xlo; ix < xhi; ++ix) {
        const size_t c = m.col(ix, iy);
        ColumnMasks::setRange(masks.k(c), zlo, zhi);
        if (occupied) {
          ColumnMasks::setRange(masks.o(c), zlo, zhi);
        }
      }
    }
  }
}

// ---- stage 2: candidates + clearance ---------------------------------------

void extractCandidates(
  const ColumnMasks & masks, int clearance_cells, int max_unknown, int min_known_above,
  SurfaceMap & m, BuildStats & stats)
{
  const size_t ncols = m.columns();
  m.col_begin.assign(ncols + 1, 0);
  for (size_t c = 0; c < ncols; ++c) {
    m.col_begin[c] = static_cast<uint32_t>(m.cells.size());
    const uint64_t * o = masks.o(c);
    const uint64_t * k = masks.k(c);
    for (int b = 0; b < masks.words; ++b) {
      // occupied with a non-occupied voxel above: the top of every occupied run
      const uint64_t above = (o[b] >> 1) | (b + 1 < masks.words ? o[b + 1] << 63 : 0ULL);
      uint64_t cand = o[b] & ~above;
      while (cand) {
        const int kz = (b << 6) + __builtin_ctzll(cand);
        cand &= cand - 1;
        if (kz >= masks.nz) {
          break;
        }
        const int zlo = kz + 1;
        const int zhi = std::min(kz + 1 + clearance_cells, masks.nz);
        SurfaceCell cell{};
        cell.kz = static_cast<int16_t>(kz);
        cell.component = kNoComponent;
        cell.clearance_d = 0.f;
        // Voxels above the grid top were never observed either.
        const int known = zlo < zhi ? ColumnMasks::popcountRange(k, zlo, zhi) : 0;
        const int unknown = clearance_cells - known;
        cell.unknown_cnt = static_cast<uint8_t>(std::min(unknown, 255));
        const int above_hi = std::min(kz + 1 + min_known_above, masks.nz);
        const int known_above = above_hi > zlo ? ColumnMasks::popcountRange(k, zlo, above_hi) : 0;
        if (zlo < zhi && ColumnMasks::anyInRange(o, zlo, zhi)) {
          cell.reject = static_cast<uint8_t>(Reject::ClearanceOccupied);
        } else if (known_above < min_known_above) {
          cell.reject = static_cast<uint8_t>(Reject::NoFreeAbove);
        } else if (unknown > max_unknown) {
          cell.reject = static_cast<uint8_t>(Reject::ClearanceUnknown);
        } else {
          cell.reject = static_cast<uint8_t>(Reject::None);
        }
        m.cells.push_back(cell);
        m.cell_col.push_back(static_cast<uint32_t>(c));
      }
    }
  }
  m.col_begin[ncols] = static_cast<uint32_t>(m.cells.size());
  stats.candidates = m.cells.size();
}

// ---- stage 3: hole fill ----------------------------------------------------

// Returns the number of cells inserted by one pass.
size_t holeFillPass(
  const ColumnMasks & masks, int clearance_cells, const std::vector<Offset> & disc, SurfaceMap & m)
{
  struct Fill
  {
    uint32_t col;
    int16_t kz;
    uint8_t unknown_cnt;
  };
  std::vector<Fill> fills;
  std::vector<std::pair<int, int>> gathered;  // (kz, quadrant)
  const int step = m.step_cells;
  const double min_support = m.params.hole_fill_min_support;
  const size_t disc_n = disc.size();

  for (int iy = 0; iy < m.ny; ++iy) {
    for (int ix = 0; ix < m.nx; ++ix) {
      const uint32_t c = m.col(ix, iy);
      gathered.clear();
      for (const Offset & o : disc) {
        const int nx_ = ix + o.dx, ny_ = iy + o.dy;
        if (!m.inGrid(nx_, ny_)) {
          continue;
        }
        const uint32_t nc = m.col(nx_, ny_);
        for (uint32_t i = m.col_begin[nc], e = m.col_begin[nc + 1]; i < e; ++i) {
          if (m.cells[i].reject == static_cast<uint8_t>(Reject::None)) {
            gathered.emplace_back(m.cells[i].kz, o.quadrant);
          }
        }
      }
      if (gathered.size() < 3) {
        continue;
      }
      // Group the heights (single linkage, gap <= step): one group per floor
      // the neighbours know about, judged on its own. Judging only the
      // largest group is wrong on a multi-storey map: under the robot's own
      // track the ground floor is missing (the lidar's blind cone) while the
      // floor above is complete, so the largest group is the one this column
      // already has and the real hole is never looked at.
      std::sort(gathered.begin(), gathered.end());
      for (size_t lo = 0; lo < gathered.size();) {
        size_t hi = lo + 1;
        while (hi < gathered.size() && gathered[hi].first - gathered[hi - 1].first <= step) {
          ++hi;
        }
        const size_t count = hi - lo;
        const size_t group_lo = lo;
        lo = hi;
        if (static_cast<double>(count) < min_support * static_cast<double>(disc_n)) {
          continue;
        }
        bool quad[4] = {false, false, false, false};
        for (size_t i = group_lo; i < hi; ++i) {
          quad[gathered[i].second] = true;
        }
        if (quad[0] + quad[1] + quad[2] + quad[3] < 3) {
          continue;  // one-sided support: a floor edge, not a hole
        }
        const int v = gathered[(group_lo + hi) / 2].first;
        // Already has a layer there (usable or not)? Then it is not a hole.
        if (m.layerNear(c, v, step + 1) != kNoCell) {
          continue;
        }
        // An occupied voxel near that height means something *was* seen
        // here: a low obstacle, a wall foot. Only an empty column is a hole.
        const int zlo = std::max(v - 1, 0);
        const int zhi = std::min(v + 1 + clearance_cells, masks.nz);
        if (zlo < zhi && ColumnMasks::anyInRange(masks.o(c), zlo, zhi)) {
          continue;
        }
        const int band_lo = v + 1, band_hi = std::min(v + 1 + clearance_cells, masks.nz);
        const int known =
          band_lo < band_hi ? ColumnMasks::popcountRange(masks.k(c), band_lo, band_hi) : 0;
        fills.push_back(
          {c, static_cast<int16_t>(v),
           static_cast<uint8_t>(std::min(clearance_cells - known, 255))});
      }
    }
  }
  for (const Fill & f : fills) {
    SurfaceCell cell{};
    cell.kz = f.kz;
    cell.flags = kInferred;
    cell.reject = static_cast<uint8_t>(Reject::None);
    cell.unknown_cnt = f.unknown_cnt;
    cell.clearance_d = 0.f;
    cell.component = kNoComponent;
    m.cells.push_back(cell);
    m.cell_col.push_back(f.col);
  }
  if (!fills.empty()) {
    rebuildCsr(m);
  }
  return fills.size();
}

// ---- stage 4: step rule ----------------------------------------------------

// A neighbour column that holds surfaces, none of them usable within one
// step, is a drop, a ledge or an obstacle: the cell is rejected. A column
// that holds no surface at all was never observed there; that is a gap,
// counted by the footprint rule, and no move goes that way anyway.
void applyStepRule(SurfaceMap & m)
{
  std::vector<uint8_t> fail(m.cells.size(), 0);
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.cells[c].reject != static_cast<uint8_t>(Reject::None)) {
      continue;
    }
    const int ix = m.colIx(m.cell_col[c]), iy = m.colIy(m.cell_col[c]);
    const int kz = m.cells[c].kz;
    for (int k = 0; k < 8; ++k) {
      const int nx_ = ix + SurfaceMap::kDx8[k], ny_ = iy + SurfaceMap::kDy8[k];
      if (!m.inGrid(nx_, ny_)) {
        fail[c] = 1;
        break;
      }
      const uint32_t nc = m.col(nx_, ny_);
      if (m.col_begin[nc] == m.col_begin[nc + 1]) {
        continue;  // a gap
      }
      const CellId n = m.layerNear(nc, kz, m.step_cells);
      if (n == kNoCell || m.cells[n].reject != static_cast<uint8_t>(Reject::None)) {
        fail[c] = 1;
        break;
      }
    }
  }
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (fail[c]) {
      m.cells[c].reject = static_cast<uint8_t>(Reject::Step);
    }
  }
}

// ---- stage 5: footprint disc + plane fit -----------------------------------

// Per cell: every column of the disc is resolved to its layer on the slope
// cone. A column with no surface at all is a gap (counted; too many reject
// the cell as FootprintGap); one whose surfaces are all out of reach or
// unusable blocks the cell (FootprintBlocked); the rest go into a
// least-squares plane whose tilt and residuals are judged. The fit is a full
// three-parameter one because with gaps the disc is no longer symmetric
// about its centre and the axes do not decouple.
void applyFootprintRule(const std::vector<Offset> & disc, SurfaceMap & m)
{
  const double tan_slope = std::tan(m.params.max_slope_deg * M_PI / 180.0);
  const double max_residual = m.res + 1e-6;
  const int max_gaps = static_cast<int>(std::floor(
    (1.0 - std::min(1.0, std::max(0.0, m.params.footprint_min_support))) * disc.size() + 1e-9));
  std::vector<uint8_t> reject(m.cells.size(), 0);
  struct Sample
  {
    double x, y, z;
  };
  std::vector<Sample> pts;
  pts.reserve(disc.size());
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (m.cells[c].reject != static_cast<uint8_t>(Reject::None)) {
      continue;
    }
    const int ix = m.colIx(m.cell_col[c]), iy = m.colIy(m.cell_col[c]);
    const int kz = m.cells[c].kz;
    const double zc = m.zTop(c);
    bool blocked = false;
    int gaps = 0;
    pts.clear();
    for (const Offset & o : disc) {
      const int nx_ = ix + o.dx, ny_ = iy + o.dy;
      if (!m.inGrid(nx_, ny_)) {
        blocked = true;  // beyond the map: unknown ground, treated as a drop
        break;
      }
      const uint32_t nc = m.col(nx_, ny_);
      if (m.col_begin[nc] == m.col_begin[nc + 1]) {
        ++gaps;
        continue;
      }
      // A neighbour on a ramp sits higher the farther out it is: the allowed
      // height difference grows along the slope cone, not just +-one step.
      const int tol = m.step_cells + static_cast<int>(std::ceil(tan_slope * o.d_m / m.res - 1e-9));
      const CellId n = m.layerNear(nc, kz, tol);
      if (n == kNoCell || m.cells[n].reject != static_cast<uint8_t>(Reject::None)) {
        blocked = true;
        break;
      }
      pts.push_back({o.dx * m.res, o.dy * m.res, m.zTop(n) - zc});
    }
    if (blocked) {
      reject[c] = static_cast<uint8_t>(Reject::FootprintBlocked);
      continue;
    }
    m.cells[c].gap_cnt = static_cast<uint8_t>(std::min(gaps, 255));
    if (gaps > max_gaps) {
      reject[c] = static_cast<uint8_t>(Reject::FootprintGap);
      continue;
    }
    // A disc whose surfaces span at most one step is level ground, with or
    // without a kerb in it: the step rule has already allowed the kerb, and
    // a plane fitted across it would read as a 30 deg slope with residuals
    // of half a step. A flight shows more than one riser in a 0.5 m disc
    // (treads are ~0.29 m) and goes on to the fit, which refuses it.
    double zlo = pts.front().z, zhi = zlo;
    for (const Sample & q : pts) {
      zlo = std::min(zlo, q.z);
      zhi = std::max(zhi, q.z);
    }
    if (zhi - zlo <= m.step_cells * m.res + 1e-9) {
      continue;
    }
    // Plane z = a x + b y + d through the samples (x, y relative to the
    // centre), by the normal equations; the centre sample is always there.
    double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
    for (const Sample & q : pts) {
      sxx += q.x * q.x;
      sxy += q.x * q.y;
      syy += q.y * q.y;
      sx += q.x;
      sy += q.y;
      sxz += q.x * q.z;
      syz += q.y * q.z;
      sz += q.z;
    }
    const double n = static_cast<double>(pts.size());
    // Solve [sxx sxy sx; sxy syy sy; sx sy n] [a b d]' = [sxz syz sz]'.
    const double det =
      sxx * (syy * n - sy * sy) - sxy * (sxy * n - sy * sx) + sx * (sxy * sy - syy * sx);
    double a = 0, b = 0, d = sz / n;
    if (std::fabs(det) > 1e-12) {
      a =
        (sxz * (syy * n - sy * sy) - sxy * (syz * n - sy * sz) + sx * (syz * sy - syy * sz)) / det;
      b =
        (sxx * (syz * n - sy * sz) - sxz * (sxy * n - sy * sx) + sx * (sxy * sz - syz * sx)) / det;
      d =
        (sxx * (syy * sz - syz * sy) - sxy * (sxy * sz - syz * sx) + sxz * (sxy * sy - syy * sx)) /
        det;
    }
    if (std::hypot(a, b) > tan_slope) {
      reject[c] = static_cast<uint8_t>(Reject::Slope);
      continue;
    }
    for (const Sample & q : pts) {
      if (std::fabs(q.z - (a * q.x + b * q.y + d)) > max_residual) {
        reject[c] = static_cast<uint8_t>(Reject::Residual);
        break;
      }
    }
  }
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (reject[c]) {
      m.cells[c].reject = reject[c];
    }
  }
}

// ---- stage 6: connected components -----------------------------------------

void labelComponents(SurfaceMap & m, BuildStats & stats)
{
  std::vector<CellId> stack;
  uint32_t next = 0;
  for (CellId seed = 0; seed < m.cells.size(); ++seed) {
    if (!m.traversable(seed) || m.cells[seed].component != kNoComponent) {
      continue;
    }
    const uint32_t id = next++;
    size_t size = 0;
    m.cells[seed].component = id;
    stack.push_back(seed);
    while (!stack.empty()) {
      const CellId c = stack.back();
      stack.pop_back();
      ++size;
      m.forEachMove(c, [&](CellId n, int) {
        if (m.cells[n].component == kNoComponent) {
          m.cells[n].component = id;
          stack.push_back(n);
        }
      });
    }
    stats.component_sizes.push_back(size);
  }
  stats.components = next;
  std::sort(stats.component_sizes.rbegin(), stats.component_sizes.rend());
}

// ---- stage 7: clearance distance field -------------------------------------

// nav2-inflation style: a traversable cell next to a non-traversable column
// seeds the queue with its distance to that column; every cell reached
// inherits the seed column and stores its own true Euclidean xy distance to
// it. Propagation runs over the surface graph (forEachMove), never across the
// 2D column grid, so a ground-floor wall does not cost the floor above it.
void computeClearance(SurfaceMap & m)
{
  const float radius = static_cast<float>(m.params.clearance_cost_radius);
  for (SurfaceCell & c : m.cells) {
    c.clearance_d = radius;
  }
  struct Item
  {
    float d;
    CellId cell;
    int src_ix, src_iy;
    bool operator>(const Item & o) const { return d > o.d; }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> q;
  for (CellId c = 0; c < m.cells.size(); ++c) {
    if (!m.traversable(c)) {
      continue;
    }
    const int ix = m.colIx(m.cell_col[c]), iy = m.colIy(m.cell_col[c]);
    const int kz = m.cells[c].kz;
    for (int k = 0; k < 8; ++k) {
      const int nx_ = ix + SurfaceMap::kDx8[k], ny_ = iy + SurfaceMap::kDy8[k];
      if (m.traversableNeighbour(nx_, ny_, kz) == kNoCell) {
        q.push(
          {static_cast<float>(std::hypot(SurfaceMap::kDx8[k], SurfaceMap::kDy8[k]) * m.res), c, nx_,
           ny_});
      }
    }
  }
  std::vector<uint8_t> done(m.cells.size(), 0);
  while (!q.empty()) {
    const Item it = q.top();
    q.pop();
    if (done[it.cell]) {
      continue;
    }
    done[it.cell] = 1;
    m.cells[it.cell].clearance_d = it.d;
    m.forEachMove(it.cell, [&](CellId n, int) {
      if (done[n]) {
        return;
      }
      const int nix = m.colIx(m.cell_col[n]), niy = m.colIy(m.cell_col[n]);
      const float d = static_cast<float>(std::hypot(nix - it.src_ix, niy - it.src_iy) * m.res);
      if (d < radius) {
        q.push({d, n, it.src_ix, it.src_iy});
      }
    });
  }
}

}  // namespace

std::shared_ptr<const SurfaceMap> buildSurfaceMap(
  const octomap::OcTree & tree, const SurfaceMapParams & params, BuildStats * stats_out)
{
  BuildStats local;
  BuildStats & stats = stats_out ? *stats_out : local;
  stats = BuildStats{};
  const auto t_start = Clock::now();

  auto map = std::make_shared<SurfaceMap>();
  SurfaceMap & m = *map;
  m.params = params;
  m.res = tree.getResolution();
  m.step_cells = std::max(0, static_cast<int>(std::lround(params.max_step_height / m.res)));
  const int clearance_cells =
    std::max(1, static_cast<int>(std::ceil(params.body_clearance / m.res - 1e-9)));
  const int max_unknown = params.max_unknown_clearance_cells < 0
                            ? clearance_cells
                            : std::min(params.max_unknown_clearance_cells, clearance_cells);

  // Grid = the tree's metric bounding box (which includes whole pruned
  // blocks). The +-res/2 lands coordToKey in the centre of the edge cells,
  // because the bounds sit exactly on cell faces where floor() can go either way.
  double xmin, ymin, zmin, xmax, ymax, zmax;
  tree.getMetricMin(xmin, ymin, zmin);
  tree.getMetricMax(xmax, ymax, zmax);
  if (tree.size() == 0 || xmax - xmin < m.res || ymax - ymin < m.res || zmax - zmin < m.res) {
    m.col_begin.assign(1, 0);
    stats.t_total = since(t_start);
    return map;
  }
  octomap::OcTreeKey k0;
  k0[0] = tree.coordToKey(xmin + m.res / 2);
  k0[1] = tree.coordToKey(ymin + m.res / 2);
  k0[2] = tree.coordToKey(zmin + m.res / 2);
  m.nx = static_cast<int>(tree.coordToKey(xmax - m.res / 2)) - static_cast<int>(k0[0]) + 1;
  m.ny = static_cast<int>(tree.coordToKey(ymax - m.res / 2)) - static_cast<int>(k0[1]) + 1;
  m.nz = static_cast<int>(tree.coordToKey(zmax - m.res / 2)) - static_cast<int>(k0[2]) + 1;
  m.x0 = tree.keyToCoord(k0[0]) - m.res / 2;
  m.y0 = tree.keyToCoord(k0[1]) - m.res / 2;
  m.z0 = tree.keyToCoord(k0[2]) - m.res / 2;
  stats.columns = m.columns();

  auto t = Clock::now();
  std::shared_ptr<ColumnMasks> masks;
  {
    masks = std::make_shared<ColumnMasks>(m.columns(), m.nz);
    fillColumnMasks(tree, m, k0, *masks, stats);
    stats.t_columns = since(t);

    t = Clock::now();
    extractCandidates(
      *masks, clearance_cells, max_unknown,
      std::max(0, std::min(params.min_known_free_above, clearance_cells)), m, stats);
    stats.t_candidates = since(t);

    t = Clock::now();
    const auto fill_disc = discOffsets(params.hole_fill_radius, m.res, false);
    for (int pass = 0; pass < params.hole_fill_passes; ++pass) {
      const size_t n = holeFillPass(*masks, clearance_cells, fill_disc, m);
      stats.inferred += n;
      if (n == 0) {
        break;
      }
    }
    stats.t_hole_fill = since(t);
    masks.reset();  // 60 MB on a site map; nothing below needs the voxels
  }

  t = Clock::now();
  applyStepRule(m);
  stats.t_step = since(t);

  t = Clock::now();
  const auto footprint = discOffsets(params.robot_radius, m.res, true);
  m.footprint_n = static_cast<int>(footprint.size());
  applyFootprintRule(footprint, m);
  stats.t_footprint = since(t);

  for (SurfaceCell & c : m.cells) {
    if (c.reject == static_cast<uint8_t>(Reject::None)) {
      c.flags |= kTraversable;
      ++stats.traversable;
    } else {
      ++stats.rejected[c.reject];
    }
  }

  t = Clock::now();
  labelComponents(m, stats);
  stats.t_components = since(t);

  t = Clock::now();
  computeClearance(m);
  stats.t_clearance = since(t);

  stats.t_total = since(t_start);
  return map;
}

}  // namespace syncai_global_planner::octomap_planner_3d
