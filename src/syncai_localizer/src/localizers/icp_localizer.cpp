#include "syncai_localizer/localizers/icp_localizer.h"

#include <rclcpp/logging.hpp>

#include <cmath>
#include <iostream>

namespace
{
// For logging: prints a 4x4 pose's translation (m) and ZYX Euler angles (deg), so the jumps of the
// ICP solution can be compared round by round.
void logPose(const char * tag, const M4F & T)
{
  const float rad2deg = 180.f / static_cast<float>(M_PI);
  const float roll = std::atan2(T(2, 1), T(2, 2)) * rad2deg;
  const float pitch = std::asin(-T(2, 0)) * rad2deg;
  const float yaw = std::atan2(T(1, 0), T(0, 0)) * rad2deg;
  RCLCPP_INFO(
    rclcpp::get_logger("icp_localizer"), "%s: t=(%.3f, %.3f, %.3f)m rpy=(%.2f, %.2f, %.2f)deg", tag,
    T(0, 3), T(1, 3), T(2, 3), roll, pitch, yaw);
}

// For logging: prints the convergence status of one registration stage. fitness is
// pcl::Registration's getFitnessScore() (the mean squared nearest-neighbor distance from the
// registered source to the target, in m²), which does not depend on the algorithm, so the physical
// meaning of *_score_thresh is unchanged after the switch to GICP; error / num_inliers are
// small_gicp's own Mahalanobis residual and inlier count, used to tell, when the threshold blocks
// an update, whether the scan does not match or there simply are not enough inliers.
void logResult(const char * tag, const small_gicp::RegistrationResult & r, double fitness)
{
  RCLCPP_INFO(
    rclcpp::get_logger("icp_localizer"),
    "%s: converged=%s iters=%zu inliers=%zu error=%.4f fitness=%.4f", tag,
    r.converged ? "yes" : "no", r.iterations, r.num_inliers, r.error, fitness);
}
}  // namespace

ICPLocalizer::ICPLocalizer(const ICPConfig & config) : m_config(config)
{
  m_refine_inp.reset(new CloudType);
  m_refine_tgt.reset(new CloudType);
  m_rough_inp.reset(new CloudType);
  m_rough_tgt.reset(new CloudType);

  // setNumThreads() must be set before the first setInputTarget() (loadMap): RegistrationPCL
  // builds its KdTreeBuilderOMP at setInputTarget time with the num_threads_ current then, and
  // changing it afterwards does not rebuild that tree.
  m_rough_icp.setNumThreads(m_config.num_threads);
  m_refine_icp.setNumThreads(m_config.num_threads);
  m_rough_icp.setCorrespondenceRandomness(m_config.num_neighbors);
  m_refine_icp.setCorrespondenceRandomness(m_config.num_neighbors);
  m_rough_icp.setRegistrationType(m_config.rough_registration_type);
  m_refine_icp.setRegistrationType(m_config.refine_registration_type);
  m_rough_icp.setVoxelResolution(m_config.rough_voxel_resolution);
  m_refine_icp.setVoxelResolution(m_config.refine_voxel_resolution);

  // Without this, RegistrationPCL's default is 1000 m, i.e. effectively no upper bound: when the
  // guess is far off, every scan point gets forced onto some map point, and the distant wrong
  // pairs take part in the optimisation, risking convergence to a wrong pose. rough gets a larger
  // convergence range to absorb the error of a hand-entered relocalize guess; refine tightens it
  // for the fine registration.
  m_rough_icp.setMaxCorrespondenceDistance(m_config.rough_max_corr_dist);
  m_refine_icp.setMaxCorrespondenceDistance(m_config.refine_max_corr_dist);
}

bool ICPLocalizer::loadMap(const std::string & path)
{
  if (!std::filesystem::exists(path)) {
    std::cerr << "Map file not found: " << path << std::endl;
    return false;
  }
  pcl::PCDReader reader;
  CloudType::Ptr cloud(new CloudType);
  reader.read(path, *cloud);

  // The slow part (file IO + downsampling) works on local clouds with a
  // local filter, so align() keeps running on the previous map meanwhile;
  // only the pointer swap at the end takes the lock. m_voxel_filter is not
  // used here because setInput() may be using it on the timer thread.
  CloudType::Ptr refine_tgt(new CloudType);
  CloudType::Ptr rough_tgt(new CloudType);
  pcl::VoxelGrid<PointType> voxel_filter;
  if (m_config.refine_map_resolution > 0) {
    voxel_filter.setLeafSize(
      m_config.refine_map_resolution, m_config.refine_map_resolution,
      m_config.refine_map_resolution);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*refine_tgt);
  } else {
    pcl::copyPointCloud(*cloud, *refine_tgt);
  }

  if (m_config.rough_map_resolution > 0) {
    voxel_filter.setLeafSize(
      m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.filter(*rough_tgt);
  } else {
    pcl::copyPointCloud(*cloud, *rough_tgt);
  }

  if (refine_tgt->empty() || rough_tgt->empty()) {
    std::cerr << "Map file is empty: " << path << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(m_target_mutex);
  m_refine_tgt.swap(refine_tgt);
  m_rough_tgt.swap(rough_tgt);
  // The target is set only here, once. The whole map gets built into two trees: RegistrationPCL
  // builds its own small_gicp KdTree at setInputTarget time (multi-threaded, used for
  // registration), and pcl::Registration builds its FLANN tree on the first align afterwards
  // (initCompute checks target_cloud_updated_; only getFitnessScore uses it). Both are built only
  // once; calling setInputTarget on every align would rebuild the whole map's trees twice per
  // cycle. The target's GICP covariances are likewise estimated once over the whole map on the
  // first align and then cached.
  m_rough_icp.setInputTarget(m_rough_tgt);
  m_refine_icp.setInputTarget(m_refine_tgt);
  return true;
}

void ICPLocalizer::setInput(const CloudType::Ptr & cloud)
{
  // Allocate a fresh cloud every round; the new scan must not be filtered into the same reused
  // Ptr: the first line of RegistrationPCL::setInputSource() is `if (input_ == cloud) return;`,
  // so an identical pointer skips everything — the source KdTree is not rebuilt and the previous
  // round's source covariances are not cleared (computeTransformation only re-estimates them when
  // the size differs), so GICP would register the new scan with the old scan's covariances.
  // pcl::IterativeClosestPoint has no such early return, which is why the original
  // buffer-reusing code was safe under it.
  m_refine_inp.reset(new CloudType);
  m_rough_inp.reset(new CloudType);

  if (m_config.refine_scan_resolution > 0) {
    m_voxel_filter.setLeafSize(
      m_config.refine_scan_resolution, m_config.refine_scan_resolution,
      m_config.refine_scan_resolution);
    m_voxel_filter.setInputCloud(cloud);
    m_voxel_filter.filter(*m_refine_inp);
  } else {
    pcl::copyPointCloud(*cloud, *m_refine_inp);
  }

  if (m_config.rough_scan_resolution > 0) {
    m_voxel_filter.setLeafSize(
      m_config.rough_scan_resolution, m_config.rough_scan_resolution,
      m_config.rough_scan_resolution);
    m_voxel_filter.setInputCloud(cloud);
    m_voxel_filter.filter(*m_rough_inp);
  } else {
    pcl::copyPointCloud(*cloud, *m_rough_inp);
  }
}

bool ICPLocalizer::align(M4F & guess)
{
  // held for the whole alignment so loadMap() cannot swap the targets
  // mid-ICP; worst case a pending swap waits one align (~tens of ms)
  std::lock_guard<std::mutex> lock(m_target_mutex);
  CloudType::Ptr aligned_cloud(new CloudType);

  // coarse-to-fine strategy; the target was already set in loadMap(), only the source is swapped
  // here, so an align never triggers a rebuild of the target KD-tree.
  if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0) return false;

  logPose("guess ", guess);

  m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
  m_rough_icp.setInputSource(m_rough_inp);
  m_rough_icp.align(*aligned_cloud, guess);
  double rough_score = m_rough_icp.getFitnessScore();
  // Both stages log before the threshold check: after the switch to GICP, hasConverged() went from
  // "almost always true" to something that really does block (see the note on
  // ICPConfig::refine_max_iteration), so when localization is stuck it must be obvious at a glance
  // whether converged failed or the score did.
  logResult("rough ", m_rough_icp.getRegistrationResult(), rough_score);
  if (!m_rough_icp.hasConverged() || rough_score > m_config.rough_score_thresh) return false;
  m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
  m_refine_icp.setInputSource(m_refine_inp);
  m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
  double refine_score = m_refine_icp.getFitnessScore();
  logResult("refine", m_refine_icp.getRegistrationResult(), refine_score);
  if (!m_refine_icp.hasConverged() || refine_score > m_config.refine_score_thresh) return false;
  guess = m_refine_icp.getFinalTransformation();

  return true;
}
