#pragma once
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <cstdint>

#include "syncai_pointlio/map_builder/commons.h"
#include "syncai_pointlio/map_builder/ivox_map.h"
#include "syncai_pointlio/map_builder/point_ekf.h"

// One group of points sharing the same per-point timestamp (curvature).
// Indices are [begin, end) into the downsampled scan; time is absolute.
struct PointGroup
{
  double time;
  int begin;
  int end;
};

// What the point-to-plane update of one scan did, for the node's health log
// (2026-11-02). On 2026-10-30 robot01's Point-LIO diverged and nothing it
// logged said how well its scans had been matching beforehand; these are the
// numbers that would have. Reset by preprocess(), filled by updateChunk().
//  - points: the downsampled scan, what the update sees.
//  - near: points with near_search_num map neighbours, the farthest within
//    sqrt(5) m. Few means the scan does not land on the map -- the pose is
//    already wrong, or the place is new.
//  - effective: points of `near` whose neighbours fit a plane (plane_thr) and
//    that passed the residual gate, i.e. the rows of the update. Near but not
//    effective means surroundings that are not planar (grass, foliage,
//    people).
//  - residual_sum: sum of |n^T p + d| of the effective points before their
//    update, metres.
//  - chunks / chunks_updated: update chunks, and those with at least one
//    effective point. A chunk without one is IMU propagation only.
struct LidarUpdateStats
{
  int points = 0;
  int near = 0;
  int effective = 0;
  double residual_sum = 0.0;
  int chunks = 0;
  int chunks_updated = 0;
};

class LidarProcessor
{
public:
  LidarProcessor(Config & config, std::shared_ptr<PointEKF> kf);

  // Offer the last scan's points, in the world frame, to the map, which keeps
  // the first point in each voxel and refuses the rest.
  void incrCloudMap();

  // Seed the map with the first scan, under the same rule.
  void initCloudMap(PointVec & point_vec);

  // Downsample the scan, re-sort by per-point time and split it into
  // same-timestamp groups (Point-LIO "time compressing").
  void preprocess(SyncPackage & package, Vec<PointGroup> & groups);

  // Point-to-plane update of one group at its own timestamp; groups larger
  // than batch_max_points are split into sequential chunks.
  void processGroup(const PointGroup & group);

  // The last preprocess() + processGroup() calls' numbers.
  const LidarUpdateStats & stats() const { return m_stats; }
  const IVoxMap & map() const { return *m_map; }

  static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D & r, const V3D & t);
  M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
  V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

private:
  void updateChunk(int begin, int end);

  Config m_config;
  std::shared_ptr<PointEKF> m_kf;
  std::shared_ptr<IVoxMap> m_map;
  CloudType::Ptr m_cloud_down_lidar;
  CloudType::Ptr m_cloud_down_world;
  // One byte per point, NOT std::vector<bool>: updateChunk writes these
  // from MP_PROC_NUM OpenMP threads, one index each, and vector<bool> packs
  // 64 flags into one word, so two threads writing neighbouring indices
  // race on a read-modify-write of the same word and one write is lost.
  // A flag left `true` that way sends the point into the update with its
  // m_norm_vec entry from an earlier scan -- a residual against a plane it
  // was never matched to, at lidar_meas_cov's full weight. Distinct bytes
  // are distinct memory locations, so per-index writes do not race.
  // Upstream FAST-LIO / Point-LIO use a plain bool array; the vector<bool>
  // came in with the port (found 2026-10-30 reviewing the robot01
  // divergence; not shown to be its cause).
  std::vector<uint8_t> m_point_selected_flag;
  CloudType::Ptr m_norm_vec;
  std::vector<PointVec> m_nearest_points;
  pcl::VoxelGrid<PointType> m_scan_filter;
  HMatX12D m_H;
  Eigen::VectorXd m_z;
  LidarUpdateStats m_stats;
};
