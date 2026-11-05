#include "syncai_pointlio/map_builder/ivox_map.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

IVoxMap::IVoxMap(double grid_resolution, std::size_t capacity, int nearby)
: m_inv_resolution(1.0 / grid_resolution), m_capacity(capacity)
{
  m_nearby.push_back(Key(0, 0, 0));
  if (nearby >= 6) {
    for (int axis = 0; axis < 3; axis++)
      for (int s : {-1, 1}) {
        Key k = Key::Zero();
        k(axis) = s;
        m_nearby.push_back(k);
      }
  }
  if (nearby >= 18) {
    // Edges: two axes off by one, the third zero.
    for (int a = 0; a < 3; a++)
      for (int b = a + 1; b < 3; b++)
        for (int sa : {-1, 1})
          for (int sb : {-1, 1}) {
            Key k = Key::Zero();
            k(a) = sa;
            k(b) = sb;
            m_nearby.push_back(k);
          }
  }
  if (nearby >= 26) {
    for (int sx : {-1, 1})
      for (int sy : {-1, 1})
        for (int sz : {-1, 1}) m_nearby.push_back(Key(sx, sy, sz));
  }
}

std::size_t IVoxMap::KeyHash::operator()(const Key & k) const
{
  // The usual spatial hash (Teschner et al. 2003), as faster-lio's hash_vec.
  return static_cast<std::size_t>(
    (static_cast<int64_t>(k.x()) * 73856093) ^ (static_cast<int64_t>(k.y()) * 471943) ^
    (static_cast<int64_t>(k.z()) * 83492791));
}

IVoxMap::Key IVoxMap::keyOf(const PointType & p) const
{
  return Key(
    static_cast<int>(std::floor(p.x * m_inv_resolution)),
    static_cast<int>(std::floor(p.y * m_inv_resolution)),
    static_cast<int>(std::floor(p.z * m_inv_resolution)));
}

void IVoxMap::addPoints(const PointVec & points)
{
  for (const PointType & p : points) {
    const Key key = keyOf(p);
    auto it = m_grids.find(key);
    if (it == m_grids.end()) {
      m_lru.push_front(Grid{key, PointVec()});
      it = m_grids.emplace(key, m_lru.begin()).first;
    } else {
      m_lru.splice(m_lru.begin(), m_lru, it->second);
    }
    it->second->points.push_back(p);
    ++m_num_points;
  }
  // After the whole batch, not per point as upstream does: a grid created
  // by this scan is at the front and never the one dropped.
  while (m_grids.size() > m_capacity) {
    const Grid & oldest = m_lru.back();
    m_num_points -= oldest.points.size();
    m_grids.erase(oldest.key);
    m_lru.pop_back();
    ++m_num_evicted;
  }
}

void IVoxMap::nearestSearch(
  const PointType & point, int k, double max_range, PointVec & nearest,
  std::vector<float> & distances) const
{
  nearest.clear();
  distances.clear();
  if (k <= 0) return;
  const float max_sq = static_cast<float>(max_range * max_range);
  const Key key = keyOf(point);
  std::vector<std::pair<float, const PointType *>> candidates;
  candidates.reserve(static_cast<std::size_t>(k) * m_nearby.size());
  for (const Key & delta : m_nearby) {
    auto it = m_grids.find(key + delta);
    if (it == m_grids.end()) continue;
    for (const PointType & q : it->second->points) {
      const float d = sq_dist(q, point);
      if (d < max_sq) candidates.emplace_back(d, &q);
    }
  }
  auto by_dist = [](
                   const std::pair<float, const PointType *> & a,
                   const std::pair<float, const PointType *> & b) { return a.first < b.first; };
  // Fully sorted, where upstream only puts the nearest first: the callers
  // read the k-th distance for the sqrt(5) m gate and points_near[0] as the
  // nearest.
  if (candidates.size() > static_cast<std::size_t>(k)) {
    std::partial_sort(candidates.begin(), candidates.begin() + k, candidates.end(), by_dist);
    candidates.resize(k);
  } else {
    std::sort(candidates.begin(), candidates.end(), by_dist);
  }
  nearest.reserve(candidates.size());
  distances.reserve(candidates.size());
  for (const auto & c : candidates) {
    nearest.push_back(*c.second);
    distances.push_back(c.first);
  }
}
