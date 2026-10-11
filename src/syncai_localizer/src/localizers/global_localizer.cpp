#include "syncai_localizer/localizers/global_localizer.h"

#include <rclcpp/logging.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <sstream>

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("global_localizer"); }

double secondsSince(GlobalLocalizer::Clock::time_point t0)
{
  return std::chrono::duration<double>(GlobalLocalizer::Clock::now() - t0).count();
}

// Signed yaw difference wrapped to [-pi, pi].
double yawDiff(double a, double b) { return std::remainder(a - b, 2.0 * M_PI); }

double yawOf(const M4F & T) { return std::atan2(T(1, 0), T(0, 0)); }

CloudType::Ptr voxelize(const CloudType::Ptr & cloud, double res)
{
  CloudType::Ptr out(new CloudType);
  if (res <= 0) {
    pcl::copyPointCloud(*cloud, *out);
    return out;
  }
  pcl::VoxelGrid<PointType> filter;
  filter.setLeafSize(res, res, res);
  filter.setInputCloud(cloud);
  filter.filter(*out);
  return out;
}

struct Candidate
{
  V3D position;
  double yaw = 0.0;
  M4F guess() const
  {
    M4F T = M4F::Identity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisf(static_cast<float>(yaw), V3F::UnitZ()).toRotationMatrix();
    T.block<3, 1>(0, 3) = position.cast<float>();
    return T;
  }
};

struct FineResult
{
  M4F pose = M4F::Identity();
  double fitness = 0.0;
  double inlier_ratio = 0.0;
};
}  // namespace

struct GlobalLocalizer::Tree
{
  explicit Tree(const CloudType::Ptr & cloud, int num_threads)
  : kdtree(cloud, small_gicp::KdTreeBuilderOMP(num_threads))
  {
  }

  // Fraction of `cloud` (transformed by T) whose nearest map point is within
  // max_dist. Read-only on the tree, so safe to call from several threads.
  double inlierRatio(const std::vector<V3D> & cloud, const M4F & T, double max_dist) const
  {
    if (cloud.empty()) return 0.0;
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0).cast<double>();
    const V3D t = T.block<3, 1>(0, 3).cast<double>();
    const double max_sq = max_dist * max_dist;
    size_t inliers = 0;
    for (const V3D & p : cloud) {
      Eigen::Vector4d q;
      q << R * p + t, 1.0;
      size_t idx;
      double sq;
      if (kdtree.nearest_neighbor_search(q, &idx, &sq) == 1 && sq < max_sq) ++inliers;
    }
    return static_cast<double>(inliers) / static_cast<double>(cloud.size());
  }

  small_gicp::KdTree<CloudType> kdtree;
};

namespace
{
std::vector<V3D> toPoints(const CloudType & cloud, size_t max_points)
{
  // A deterministic stride rather than a random sample, so a bag replay gives
  // the same coarse ranking every time.
  const size_t stride =
    (max_points > 0 && cloud.size() > max_points) ? (cloud.size() + max_points - 1) / max_points : 1;
  std::vector<V3D> out;
  out.reserve(cloud.size() / stride + 1);
  for (size_t i = 0; i < cloud.size(); i += stride) {
    out.emplace_back(cloud[i].x, cloud[i].y, cloud[i].z);
  }
  return out;
}
}  // namespace

GlobalLocalizer::GlobalLocalizer(const GlobalLocConfig & config, const ICPConfig & icp_config)
: m_config(config), m_icp_config(icp_config)
{
  // Same backend settings as ICPLocalizer's stages (thread count before the
  // first setInputTarget, for the same KdTreeBuilderOMP reason), except the
  // rough correspondence radius and the iteration budget -- see
  // GlobalLocConfig::rough_max_corr_dist.
  for (auto * icp : {&m_rough_icp, &m_refine_icp}) {
    icp->setNumThreads(m_icp_config.num_threads);
    icp->setCorrespondenceRandomness(m_icp_config.num_neighbors);
    icp->setMaximumIterations(m_config.max_iteration);
  }
  m_rough_icp.setRegistrationType(m_icp_config.rough_registration_type);
  m_rough_icp.setVoxelResolution(m_icp_config.rough_voxel_resolution);
  m_rough_icp.setMaxCorrespondenceDistance(m_config.rough_max_corr_dist);
  m_rough_icp.setRotationEpsilon(m_icp_config.rough_rotation_eps);
  m_rough_icp.setTransformationEpsilon(m_icp_config.rough_translation_eps);

  m_refine_icp.setRegistrationType(m_icp_config.refine_registration_type);
  m_refine_icp.setVoxelResolution(m_icp_config.refine_voxel_resolution);
  m_refine_icp.setMaxCorrespondenceDistance(m_icp_config.refine_max_corr_dist);
  m_refine_icp.setRotationEpsilon(m_icp_config.refine_rotation_eps);
  m_refine_icp.setTransformationEpsilon(m_icp_config.refine_translation_eps);
}

bool GlobalLocalizer::loadCandidatePositions(
  const std::string & poses_path, std::vector<V3D> & positions, std::string & error) const
{
  positions.clear();
  std::ifstream in(poses_path);
  if (!in) {
    error = "cannot read " + poses_path +
            " (auto_localize needs a map saved with save_patches)";
    return false;
  }
  // Greedy thinning in trajectory order. O(lines x kept), which is a few
  // thousand x a few hundred on the largest site so far -- not worth a grid.
  const double min_sq = m_config.candidate_spacing * m_config.candidate_spacing;
  std::string line;
  size_t parsed = 0;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string name;
    double x, y, z;
    if (!(ss >> name >> x >> y >> z)) continue;
    ++parsed;
    const V3D p(x, y, z);
    const bool near_kept = std::any_of(
      positions.begin(), positions.end(),
      [&](const V3D & k) { return (k - p).squaredNorm() < min_sq; });
    if (!near_kept) positions.push_back(p);
  }
  if (positions.empty()) {
    error = "no keyframe pose in " + poses_path;
    return false;
  }
  RCLCPP_INFO(
    logger(), "candidates: %zu keyframes in %s thinned to %zu positions at %.2f m", parsed,
    poses_path.c_str(), positions.size(), m_config.candidate_spacing);
  return true;
}

void GlobalLocalizer::setTargets(const CloudType::Ptr & rough_map, const CloudType::Ptr & refine_map)
{
  if (rough_map != m_rough_map) {
    const auto t0 = Clock::now();
    m_rough_map = rough_map;
    m_rough_icp.setInputTarget(m_rough_map);
    m_rough_tree = std::make_shared<Tree>(m_rough_map, m_icp_config.num_threads);
    RCLCPP_INFO(
      logger(), "rough target rebuilt: %zu points, %.2f s", m_rough_map->size(), secondsSince(t0));
  }
  if (refine_map != m_refine_map) {
    const auto t0 = Clock::now();
    m_refine_map = refine_map;
    m_refine_icp.setInputTarget(m_refine_map);
    m_refine_tree = std::make_shared<Tree>(m_refine_map, m_icp_config.num_threads);
    RCLCPP_INFO(
      logger(), "refine target rebuilt: %zu points, %.2f s", m_refine_map->size(),
      secondsSince(t0));
  }
}

GlobalLocResult GlobalLocalizer::search(
  const CloudType::Ptr & scan, const std::vector<V3D> & positions,
  const CloudType::Ptr & rough_map, const CloudType::Ptr & refine_map, Clock::time_point deadline)
{
  GlobalLocResult result;
  const auto t_start = Clock::now();
  const bool use_deadline = m_config.search_timeout_sec > 0;
  auto expired = [&]() { return use_deadline && Clock::now() > deadline; };
  auto timeout = [&](const std::string & where) {
    result.success = false;
    result.has_pose = false;
    result.message = "search timed out after " + std::to_string(secondsSince(t_start)) + " s " +
                     where + " (auto_loc.search_timeout_sec " +
                     std::to_string(m_config.search_timeout_sec) + ")";
    RCLCPP_WARN(logger(), "%s", result.message.c_str());
    return result;
  };

  setTargets(rough_map, refine_map);
  if (expired()) return timeout("while building the map targets");

  const CloudType::Ptr rough_scan = voxelize(scan, m_icp_config.rough_scan_resolution);
  const CloudType::Ptr refine_scan = voxelize(scan, m_icp_config.refine_scan_resolution);
  const std::vector<V3D> coarse_pts = toPoints(
    *voxelize(scan, m_config.coarse_scan_resolution),
    static_cast<size_t>(std::max(0, m_config.coarse_max_points)));
  const std::vector<V3D> refine_pts = toPoints(*refine_scan, 0);
  if (coarse_pts.empty() || rough_scan->empty() || refine_scan->empty()) {
    result.message = "accumulated scan is empty";
    return result;
  }

  // ---- Stage 1: coarse inlier ratio of every (position, yaw) candidate.
  const int yaw_steps = std::max(1, static_cast<int>(std::lround(360.0 / m_config.yaw_step_deg)));
  const double yaw_step = 2.0 * M_PI / yaw_steps;
  std::vector<Candidate> candidates;
  candidates.reserve(positions.size() * yaw_steps);
  for (const V3D & p : positions) {
    for (int k = 0; k < yaw_steps; ++k) candidates.push_back({p, k * yaw_step - M_PI});
  }
  result.candidates = static_cast<int>(candidates.size());

  std::vector<double> coarse(candidates.size(), -1.0);
  std::atomic<bool> timed_out{false};
  const auto t_coarse = Clock::now();
  // Parallel over candidates; KD-tree queries are read-only. A thread that
  // sees the deadline pass flags it and the rest skip their remaining work --
  // an OpenMP loop cannot break.
#pragma omp parallel for num_threads(m_icp_config.num_threads) schedule(dynamic, 32)
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    if (timed_out.load(std::memory_order_relaxed)) continue;
    if (i % 32 == 0 && expired()) {
      timed_out = true;
      continue;
    }
    coarse[i] =
      m_rough_tree->inlierRatio(coarse_pts, candidates[i].guess(), m_config.coarse_inlier_dist);
  }
  if (timed_out) {
    return timeout("in the coarse stage (" + std::to_string(candidates.size()) + " candidates)");
  }

  // Top-k with non-max suppression: without it the top_k are mostly one place
  // at neighbouring yaws, and the ambiguity check below would never see an
  // alternative to doubt the winner with. The radius is at least 1.5 grid /
  // yaw steps, so direct neighbours on the grid always collapse.
  std::vector<int> order(candidates.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&](int a, int b) { return coarse[a] > coarse[b]; });
  const double nms_trans = std::max(m_config.ambiguity_trans, 1.5 * m_config.candidate_spacing);
  const double nms_yaw = std::max(m_config.ambiguity_yaw_deg * M_PI / 180.0, 1.5 * yaw_step);
  std::vector<int> selected;
  for (int i : order) {
    if (static_cast<int>(selected.size()) >= m_config.top_k) break;
    const Candidate & c = candidates[i];
    const bool suppressed = std::any_of(selected.begin(), selected.end(), [&](int s) {
      return (candidates[s].position - c.position).norm() < nms_trans &&
             std::abs(yawDiff(candidates[s].yaw, c.yaw)) < nms_yaw;
    });
    if (!suppressed) selected.push_back(i);
  }
  RCLCPP_INFO(
    logger(),
    "coarse: %zu candidates (%zu positions x %d yaws) x %zu points in %.2f s, best %.3f, "
    "%zu kept for GICP",
    candidates.size(), positions.size(), yaw_steps, coarse_pts.size(), secondsSince(t_coarse),
    coarse[order.front()], selected.size());

  // ---- Stage 2: rough + refine GICP of each kept candidate.
  // The source is the same cloud for every candidate, so RegistrationPCL's
  // `input_ == cloud` early return in setInputSource keeps its covariances
  // cached across them -- exactly what is wanted here (the opposite of the
  // timer's case, see ICPLocalizer::setInput).
  m_rough_icp.setInputSource(rough_scan);
  m_refine_icp.setInputSource(refine_scan);
  CloudType aligned;
  std::vector<FineResult> fine;
  const auto t_fine = Clock::now();
  for (int i : selected) {
    if (expired()) return timeout("in the GICP stage");
    m_rough_icp.align(aligned, candidates[i].guess());
    if (expired()) return timeout("in the GICP stage");
    m_refine_icp.align(aligned, m_rough_icp.getFinalTransformation());
    FineResult r;
    r.pose = m_refine_icp.getFinalTransformation();
    r.fitness = m_refine_icp.getFitnessScore();
    r.inlier_ratio = m_refine_tree->inlierRatio(refine_pts, r.pose, m_config.refine_inlier_dist);
    RCLCPP_INFO(
      logger(),
      "fine: start (%.2f, %.2f, %.0f deg) coarse %.3f -> (%.2f, %.2f, %.1f deg) "
      "refine converged=%s fitness=%.4f inliers=%.3f",
      candidates[i].position.x(), candidates[i].position.y(), candidates[i].yaw * 180.0 / M_PI,
      coarse[i], r.pose(0, 3), r.pose(1, 3), yawOf(r.pose) * 180.0 / M_PI,
      m_refine_icp.hasConverged() ? "yes" : "no", r.fitness, r.inlier_ratio);
    fine.push_back(r);
  }
  if (fine.empty()) {
    result.message = "no candidate survived the coarse stage";
    return result;
  }

  // ---- Pick and check.
  auto better = [](const FineResult & a, const FineResult & b) {
    if (a.inlier_ratio != b.inlier_ratio) return a.inlier_ratio > b.inlier_ratio;
    return a.fitness < b.fitness;
  };
  std::sort(fine.begin(), fine.end(), better);
  const FineResult & best = fine.front();
  const FineResult * runner_up = nullptr;
  for (size_t i = 1; i < fine.size(); ++i) {
    const double dt = (fine[i].pose.block<3, 1>(0, 3) - best.pose.block<3, 1>(0, 3)).norm();
    const double dyaw = std::abs(yawDiff(yawOf(fine[i].pose), yawOf(best.pose)));
    if (dt > m_config.ambiguity_trans || dyaw > m_config.ambiguity_yaw_deg * M_PI / 180.0) {
      runner_up = &fine[i];
      break;
    }
  }

  result.pose = best.pose;
  result.has_pose = true;
  result.fitness = best.fitness;
  result.inlier_ratio = best.inlier_ratio;
  char buf[256];
  std::snprintf(
    buf, sizeof(buf), "best (%.2f, %.2f, %.1f deg) inliers=%.3f fitness=%.4f, runner-up inliers=%s",
    best.pose(0, 3), best.pose(1, 3), yawOf(best.pose) * 180.0 / M_PI, best.inlier_ratio,
    best.fitness, runner_up ? std::to_string(runner_up->inlier_ratio).c_str() : "none");
  const std::string summary = buf;

  if (best.fitness > m_icp_config.refine_score_thresh) {
    result.message = "rejected, fitness above refine_score_thresh: " + summary;
  } else if (best.inlier_ratio < m_config.min_inlier_ratio) {
    result.message = "rejected, inlier ratio below auto_loc.min_inlier_ratio: " + summary;
  } else if (runner_up && runner_up->inlier_ratio >= best.inlier_ratio * m_config.ambiguity_ratio) {
    std::snprintf(
      buf, sizeof(buf), " at (%.2f, %.2f, %.1f deg)", runner_up->pose(0, 3), runner_up->pose(1, 3),
      yawOf(runner_up->pose) * 180.0 / M_PI);
    result.message = "ambiguous, a distinct pose scores alike: " + summary + buf;
  } else {
    result.success = true;
    result.message = summary;
  }
  RCLCPP_INFO(
    logger(), "search %s in %.2f s (GICP stage %.2f s): %s",
    result.success ? "succeeded" : "failed", secondsSince(t_start), secondsSince(t_fine),
    result.message.c_str());
  return result;
}
