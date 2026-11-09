#include "syncai_pointlio/map_builder/lidar_processor.h"

#include <cmath>
#ifdef MP_EN
// omp_set_num_threads() for the MP_EN residual loop.
#include <omp.h>
#endif

namespace
{
// The smallest chunk the residual loop runs on MP_PROC_NUM threads. A Livox
// CustomMsg scan after the VoxelGrid is ~1250 groups of ~1 point (a voxel's
// time is the mean of its points', so timestamps are practically unique):
// a 2-thread region over one iteration costs ~1.3 us of fork/join on the Orin
// and hands the worker nothing (static schedule), and with a new region every
// ~30 us libgomp's worker never got past its spin-wait -- robot01 showed one
// worker per executor thread at its master's CPU time, ~0.4 core of spinning
// in all (2026-11-06). The parallel loop is for PointCloud2 input
// (`lidar_type: 1`), whose one group per scan is cut into batch_max_points
// chunks. 64 is a round number well above the fork/join break-even, not tuned.
constexpr int kMinParallelChunk = 64;

// A point is matched only when its near_search_num-th map neighbour lies within this squared
// distance (upstream's 5 m^2, sqrt(5) ~ 2.24 m), so the neighbour search is bounded there too:
// nothing farther can be part of a plane the update uses, and the search's acceptance radius
// starts that small. It searched 5 m, because the voxel test once read the farther candidates.
constexpr double kMaxNeighbourSqDist = 5.0;
}  // namespace

LidarProcessor::LidarProcessor(Config & config, std::shared_ptr<PointEKF> kf)
: m_config(config), m_kf(kf)
{
  m_map = std::make_shared<IVoxMap>(
    m_config.map_grid_resolution, m_config.map_resolution, m_config.map_capacity,
    m_config.map_nearby);
  m_cloud_down_lidar.reset(new CloudType);
  m_cloud_down_world.reset(new CloudType(10000, 1));
  m_norm_vec.reset(new CloudType(10000, 1));
  m_nearest_points.resize(10000);
  m_point_selected_flag.resize(10000, false);
  m_H.resize(m_config.batch_max_points, 12);
  m_z.resize(m_config.batch_max_points);

  if (m_config.scan_resolution > 0.0) {
    m_scan_filter.setLeafSize(
      m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
  }
}

void LidarProcessor::incrCloudMap()
{
  if (m_cloud_down_lidar->empty()) return;

  // m_cloud_down_world was already transformed inside processGroup / updateChunk with the state
  // at each point group's own time (equivalent to point-by-point undistortion), so it is used
  // as is here and not recomputed.
  //
  // The map keeps the first point in each map_resolution voxel and refuses the rest
  // (IVoxMap), upstream Point-LIO's first-wins. The port had FAST-LIO2's rule instead -- keep
  // the point nearest each voxel's centre -- whose kept point is the minimum over every scan
  // ever inserted and walks off the surface over a night of laps. First-wins cannot drift that
  // way; its own cost is that whatever the first observation got wrong, a person standing there
  // included, stays until the map is reset.
  const int size = static_cast<int>(m_cloud_down_lidar->size());
  const PointVec scan_world(
    m_cloud_down_world->points.begin(), m_cloud_down_world->points.begin() + size);
  m_map->addPoints(scan_world);
}

void LidarProcessor::initCloudMap(PointVec & point_vec)
{
  // The whole first scan, through the same one-point-per-voxel rule as every later one.
  m_map->addPoints(point_vec);
}

void LidarProcessor::preprocess(SyncPackage & package, Vec<PointGroup> & groups)
{
  // The VoxelGrid downsampling voxel size is applied to this frame's point cloud before it
  // enters the EKF
  if (m_config.scan_resolution > 0.0) {
    m_scan_filter.setInputCloud(package.cloud);
    m_scan_filter.filter(*m_cloud_down_lidar);
  } else {
    pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
  }

  // The voxel filter does not preserve time order; re-sort by curvature (per-point time offset
  // in ms)
  std::sort(
    m_cloud_down_lidar->points.begin(), m_cloud_down_lidar->points.end(),
    [](const PointType & p1, const PointType & p2) { return p1.curvature < p2.curvature; });

  // These form five parallel arrays, all addressed by the same index
  int size = m_cloud_down_lidar->size();
  m_stats = LidarUpdateStats();
  m_stats.points = size;
  if (static_cast<int>(m_cloud_down_world->size()) < size) {
    m_cloud_down_world->resize(size);
    m_norm_vec->resize(size);
    m_nearest_points.resize(size);
    m_point_selected_flag.resize(size, false);
  }

  // time compressing: points sharing one timestamp form a group
  groups.clear();
  int begin = 0;
  for (int i = 1; i <= size; i++) {
    if (
      i == size ||
      m_cloud_down_lidar->points[i].curvature != m_cloud_down_lidar->points[begin].curvature) {
      double group_time =
        package.cloud_start_time + m_cloud_down_lidar->points[begin].curvature / 1000.0;
      groups.push_back({group_time, begin, i});
      begin = i;
    }
  }
}

void LidarProcessor::processGroup(const PointGroup & group)
{
  for (int chunk_begin = group.begin; chunk_begin < group.end;
       chunk_begin += m_config.batch_max_points) {
    int chunk_end = std::min(chunk_begin + m_config.batch_max_points, group.end);
    updateChunk(chunk_begin, chunk_end);
  }
}

void LidarProcessor::updateChunk(int begin, int end)
{
  const State & state = m_kf->x();
  ++m_stats.chunks;
  // A reduction, not ++m_stats.near in the loop: the loop may run on
  // MP_PROC_NUM threads, and a shared counter would lose increments the way
  // the vector<bool> flags lost writes (see m_point_selected_flag).
  int near_num = 0;
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for reduction(+ : near_num) if (end - begin >= kMinParallelChunk)
#endif
  for (int i = begin; i < end; i++) {
    PointType & point_body = m_cloud_down_lidar->points[i];
    PointType & point_world = m_cloud_down_world->points[i];
    Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
    Eigen::Vector3d point_world_vec =
      state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
    point_world.x = point_world_vec(0);
    point_world.y = point_world_vec(1);
    point_world.z = point_world_vec(2);
    point_world.intensity = point_body.intensity;
    std::vector<float> point_sq_dist;
    auto & points_near = m_nearest_points[i];
    // Fewer than near_search_num neighbours within the bound means the point is not matched.
    m_map->nearestSearch(
      point_world, m_config.near_search_num, std::sqrt(kMaxNeighbourSqDist), points_near,
      point_sq_dist);
    if (
      points_near.size() >= static_cast<size_t>(m_config.near_search_num) &&
      point_sq_dist[m_config.near_search_num - 1] <= kMaxNeighbourSqDist)
      m_point_selected_flag[i] = true;
    else
      m_point_selected_flag[i] = false;
    if (!m_point_selected_flag[i]) continue;
    ++near_num;

    Eigen::Vector4d pabcd;
    m_point_selected_flag[i] = false;
    if (esti_plane(points_near, m_config.plane_thr, pabcd)) {
      double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) +
                   pabcd(2) * point_world_vec(2) + pabcd(3);
      double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
      if (s > 0.9) {
        m_point_selected_flag[i] = true;
        m_norm_vec->points[i].x = pabcd(0);
        m_norm_vec->points[i].y = pabcd(1);
        m_norm_vec->points[i].z = pabcd(2);
        m_norm_vec->points[i].intensity = pd2;
      }
    }
  }

  int effect_num = 0;
  for (int i = begin; i < end; i++) {
    if (!m_point_selected_flag[i]) continue;
    const PointType & laser_p = m_cloud_down_lidar->points[i];
    const PointType & norm_p = m_norm_vec->points[i];
    Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
    m_H.row(effect_num).setZero();
    m_H.block<1, 3>(effect_num, 0) =
      -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_il);
    m_H.block<1, 3>(effect_num, 3) = norm_vec.transpose();
    if (m_config.esti_il) {
      m_H.block<1, 3>(effect_num, 6) =
        -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
      m_H.block<1, 3>(effect_num, 9) = norm_vec.transpose() * state.r_wi;
    }
    m_z(effect_num) = norm_p.intensity;
    m_stats.residual_sum += std::fabs(norm_p.intensity);
    effect_num++;
  }
  m_stats.near += near_num;
  m_stats.effective += effect_num;
  if (effect_num < 1) return;
  ++m_stats.chunks_updated;

  m_kf->updateLidar(m_H, m_z, effect_num);

  // Re-transform this chunk to world coordinates with the updated state, for incrCloudMap to
  // insert into the map
  const State & post = m_kf->x();
  for (int i = begin; i < end; i++) {
    const PointType & point_body = m_cloud_down_lidar->points[i];
    Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
    Eigen::Vector3d point_world_vec =
      post.r_wi * (post.r_il * point_body_vec + post.t_il) + post.t_wi;
    m_cloud_down_world->points[i].x = point_world_vec(0);
    m_cloud_down_world->points[i].y = point_world_vec(1);
    m_cloud_down_world->points[i].z = point_world_vec(2);
  }
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D & r, const V3D & t)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = r.cast<float>();
  transform.block<3, 1>(0, 3) = t.cast<float>();
  CloudType::Ptr ret(new CloudType);
  pcl::transformPointCloud(*inp, *ret, transform);
  return ret;
}
