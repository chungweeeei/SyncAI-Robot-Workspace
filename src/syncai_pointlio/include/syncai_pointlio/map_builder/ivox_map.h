#pragma once
#include <Eigen/Core>
#include <cstddef>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "syncai_pointlio/map_builder/commons.h"

// The local map the point-to-plane update matches against: an incremental
// voxel map in the shape of upstream
// Point-LIO's default branch (`point-lio-with-grid-map`, whose iVox comes from
// faster-lio). Written here rather than copied, because that code is GPL-2.0
// and this package is MIT; the behaviour is the same:
//  - Space is cut into cubic grids of `grid_resolution` (2.0 m upstream), each
//    a plain list of points, found through a hash map of integer grid keys.
//  - A k-nearest search looks only in the point's own grid and its face
//    neighbours (`nearby` 6; 18 adds the edges, 26 the corners, 0 the grid
//    alone), so it is approximate where the true neighbours sit two grids
//    away -- which the update rejects anyway past sqrt(5) m.
//  - Grids are kept in least-recently-*inserted* order, and once there are
//    more than `capacity` the oldest grid is dropped whole. A search does not
//    count as use: it runs on MP_PROC_NUM OpenMP threads and must not write.
//    Upstream's capacity (1 000 000 grids of 2 m) is a memory bound, not a
//    forgetting horizon: no site this robot patrols comes near it.
//  - Each `voxel_resolution` voxel (map_resolution, 0.2 m) holds at most one
//    point, the first that landed in it (upstream's first-wins), and the map
//    enforces that itself from a set of the voxels it holds a point in -- one
//    set for the whole map, so the rule holds whichever grid a voxel's points
//    fall in, also where a voxel spans two grids (at 0.3 m, 2 m is not a
//    multiple of the voxel). Upstream and the port judged it outside the map from
//    the update's 5 nearest neighbours, which let a voxel take a second point
//    whenever its first was not among them; on robot01 the map grew to 290
//    points per grid over 70 h and the search, which visits every point of
//    the nearby grids, slowed until scans were skipped (2026-11-08). Held
//    here, the points per grid are bounded by the voxels in it.
class IVoxMap
{
public:
  IVoxMap(double grid_resolution, double voxel_resolution, std::size_t capacity, int nearby);

  // Insert each point whose voxel holds none yet into its grid, refusing the
  // rest, moving every grid inserted into to the front of the LRU order; then
  // drop the oldest grids beyond capacity, freeing their voxels.
  void addPoints(const PointVec & points);

  // Up to k points nearest to `point` among the nearby grids, no farther
  // than max_range, in ascending distance; distances holds their squared
  // distances. Both are cleared first. Safe to call from several threads
  // at once while nothing inserts.
  void nearestSearch(
    const PointType & point, int k, double max_range, PointVec & nearest,
    std::vector<float> & distances) const;

  std::size_t numGrids() const { return m_grids.size(); }
  std::size_t numPoints() const { return m_num_points; }
  // Grids dropped by the LRU since construction.
  std::size_t numEvicted() const { return m_num_evicted; }
  // Points refused since construction because their voxel already held one.
  std::size_t numRefused() const { return m_num_refused; }

private:
  using Key = Eigen::Vector3i;
  struct KeyHash
  {
    std::size_t operator()(const Key & k) const;
  };
  struct Grid
  {
    Key key;
    PointVec points;
  };
  using GridList = std::list<Grid>;

  Key keyOf(const PointType & p) const;
  Key voxelOf(const PointType & p) const;

  double m_inv_resolution;
  double m_inv_voxel_resolution;
  std::size_t m_capacity;
  std::vector<Key> m_nearby;
  // Front = most recently inserted into.
  GridList m_lru;
  std::unordered_map<Key, GridList::iterator, KeyHash> m_grids;
  // Every voxel some grid holds a point in.
  std::unordered_set<Key, KeyHash> m_occupied_voxels;
  std::size_t m_num_points = 0;
  std::size_t m_num_evicted = 0;
  std::size_t m_num_refused = 0;
};
