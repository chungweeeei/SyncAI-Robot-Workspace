#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <mutex>
#include <string>

#include "syncai_localizer/localizers/commons.h"

// Registration was switched from pcl::IterativeClosestPoint (point-to-point) to small_gicp's GICP.
// RegistrationPCL derives from pcl::Registration, and setInputTarget / setInputSource /
// setMaximumIterations / setMaxCorrespondenceDistance / align / hasConverged /
// getFinalTransformation / getFitnessScore are all there, so the flow of align() is unchanged.
// The reason for the switch is convergence quality, not speed (a rough + refine round takes
// 30-50 ms, well under the 200 ms period of today's update_hz of 5.0):
// a MID360 scan is sparse and unevenly distributed relative to the map, and point-to-point tends
// to slide along the wall in degenerate-geometry directions such as long corridors / large flat
// walls, whereas GICP's distribution-to-distribution uses the local covariance to pin that
// direction down; it is also far more tolerant of the mismatched voxel resolutions on the scan and
// map sides (0.25/0.1).
// The headers are entirely header-only (pcl_registration.hpp includes the impl itself at the end);
// linking small_gicp::small_gicp is only for the include path and the OpenMP flags.
#include <small_gicp/pcl/pcl_registration.hpp>

struct ICPConfig
{
  double refine_scan_resolution = 0.1;
  double refine_map_resolution = 0.1;
  double refine_score_thresh = 0.1;
  // small_gicp only reports converged when the update falls below eps; hitting max_iteration
  // stops as not converged (PCL's ICP reports converged even when it hits the limit). Since
  // align() treats hasConverged() as a hard condition, too few iterations means a perfectly good
  // score still returns false every round and the TF never updates. Hence aligned with
  // small_gicp's own default of 20.
  int refine_max_iteration = 20;
  double refine_max_corr_dist = 0.5;
  // "GICP" or "VGICP". VGICP builds a voxelmap of the target instead of a KD-tree and has a
  // larger basin of attraction, which suits the rough stage's job of absorbing the error of a
  // hand-entered relocalize guess. Both stages default to GICP for now: change one variable at a
  // time, and only consider VGICP for rough once GICP's behaviour on site has been confirmed.
  std::string refine_registration_type = "GICP";
  // Only used when registration_type is VGICP.
  double refine_voxel_resolution = 0.5;

  double rough_scan_resolution = 0.25;
  double rough_map_resolution = 0.25;
  double rough_score_thresh = 0.2;
  int rough_max_iteration = 20;
  double rough_max_corr_dist = 2.0;
  std::string rough_registration_type = "GICP";
  double rough_voxel_resolution = 1.0;

  // Thread count for GICP's covariance estimation and reduction. The localizer shares one CPU
  // with pointlio, so do not use every core.
  int num_threads = 4;
  // Number of neighbors used to estimate each point's local covariance (the same as pcl::GICP's
  // correspondence randomness). Values below 5 are clamped back to 5 by small_gicp.
  int num_neighbors = 20;
};

class ICPLocalizer
{
public:
  explicit ICPLocalizer(const ICPConfig & config);

  bool loadMap(const std::string & path);

  void setInput(const CloudType::Ptr & cloud);

  bool align(M4F & guess);
  ICPConfig & config() { return m_config; }
  CloudType::Ptr roughMap()
  {
    std::lock_guard<std::mutex> lock(m_target_mutex);
    return m_rough_tgt;
  }
  CloudType::Ptr refineMap()
  {
    std::lock_guard<std::mutex> lock(m_target_mutex);
    return m_refine_tgt;
  }
  // The map only ever arrives through loadMap() (from the constructor's preload, relocalize or
  // the initialpose fallback); before it is loaded align() always fails, so sources such as
  // initialpose can check this first instead of silently swallowing the guess.
  bool isMapLoaded()
  {
    std::lock_guard<std::mutex> lock(m_target_mutex);
    return !m_refine_tgt->empty() && !m_rough_tgt->empty();
  }

private:
  ICPConfig m_config;
  // guards m_rough_tgt / m_refine_tgt: loadMap() may run on the service
  // thread while align() runs on the timer thread
  std::mutex m_target_mutex;
  pcl::VoxelGrid<PointType> m_voxel_filter;
  small_gicp::RegistrationPCL<PointType, PointType> m_refine_icp;
  small_gicp::RegistrationPCL<PointType, PointType> m_rough_icp;
  CloudType::Ptr m_refine_inp;
  CloudType::Ptr m_rough_inp;
  CloudType::Ptr m_refine_tgt;
  CloudType::Ptr m_rough_tgt;
  std::string m_pcd_path;
};
