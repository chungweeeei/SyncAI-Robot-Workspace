#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <small_gicp/pcl/pcl_registration.hpp>

#include <string>

#include "syncai_mapping/pgos/commons.h"
#include "syncai_mapping/pgos/gravity_prior_factor.h"
#include "syncai_mapping/pgos/planar_loop_factor.h"

struct KeyPoseWithCloud
{
  M3D r_local;
  V3D t_local;
  M3D r_global;
  V3D t_global;
  double time;
  CloudType::Ptr body_cloud;
};
struct LoopPair
{
  size_t source_id;
  size_t target_id;
  M3D r_offset;
  V3D t_offset;
  double score;
  // The raw ICP result that produced r_offset / t_offset: the rigid transform
  // that moves the source keyframe's WORLD-frame cloud onto the target submap.
  // Kept so pgo_node can log what the loop asked for -- in particular its z
  // component. Before 2026-10 nothing recorded this, and a run whose loop
  // closures lifted the map by 15 cm had to be diagnosed from rviz markers.
  M3D icp_r;
  V3D icp_t;
  // (dx, dy, dyaw) of the corrected source relative to the target, in the
  // WORLD frame: the measurement of PlanarLoopFactor when
  // loop_planar_correction is on. r_offset / t_offset above are what the
  // 6-DOF BetweenFactor uses when it is off.
  V3D planar_meas;
};

struct Config
{
  double key_pose_delta_deg = 10;
  double key_pose_delta_trans = 1.0;
  double loop_search_radius = 1.0;
  double loop_time_tresh = 60.0;
  double loop_score_tresh = 0.15;
  // Loop verification backend. "gicp": small_gicp's RegistrationPCL (GICP,
  // the default since 2026-10); "icp": PCL point-to-point ICP, the fork's
  // original. On the dp1f_1002 replay the PCL ICP returned 1.5-3.7 deg of
  // rotation and a z 2-7x too large for every corridor loop although the raw
  // poses already agreed to 5 cm; GICP on the same pairs returned the right
  // amount (offline check, Open3D: +0.03 m vs +0.13 m).
  std::string loop_registration = "gicp";
  // Max correspondence distance (m) for either backend. The fork hard-coded
  // 10, which in practice disables the gate: a source scan that reaches 30 m
  // down a corridor pairs its far points with whatever the submap has
  // nearest, and those pairs steer the solution. 1.0 since 2026-10.
  double loop_icp_max_corr_dist = 1.0;
  int loop_gicp_num_threads = 4;
  int loop_gicp_num_neighbors = 20;
  // Constrain only the WORLD-frame x / y / yaw between target and source
  // (PlanarLoopFactor, a 3-dim error) instead of a 6-DOF BetweenFactor. The
  // registration result is projected to x / y / yaw about the world z axis
  // first. Point-LIO's world frame is gravity-aligned and its z was
  // self-consistent to 3 cm over the dp1f_1002 run, so a loop has nothing to
  // add on z / roll / pitch -- but a 6-DOF edge with a slightly wrong tilt
  // bends the whole chain: with per-edge rotation variance 1e-6 and 30-60 m
  // of lever arm the chain is softer in z than the loop factor, and the map
  // ended up with a 15 cm double floor. An intermediate version kept the
  // BetweenFactor and merely tightened its roll / pitch / z noise; that
  // still leaked x into z through the 15 deg lidar pitch (-2..-9 cm). Off =
  // the fork's 6-DOF behaviour.
  bool loop_planar_correction = true;
  // Only with loop_planar_correction = false: BetweenFactor variance on
  // roll / pitch / z (rad^2 / m^2); x / y / yaw keep the fork's "variance =
  // registration fitness", which the planar factor uses for all three of
  // its axes as well.
  double loop_noise_var_roll_pitch_z = 1e-2;
  // Only with loop_planar_correction = true: the planar factor's yaw sigma
  // (degrees). Its x / y keep "variance = fitness". The fork's convention
  // put the fitness on yaw as well, read as rad^2: a typical 0.1-0.15 is a
  // yaw sigma of 18-22 deg, while ~500 odometry edges (rotation variance
  // 1e-6 each) accumulate about 1 deg -- so the optimiser ignored the loop's
  // yaw entirely. On dp1f_1008_2 the one loop that closed the east corridor
  // asked for 8 deg and +1.7 m and moved the keyframe 0.8 m with no yaw,
  // leaving the two passes 1.1-1.6 m / 5-6 deg apart. <= 0 = the old
  // behaviour (yaw variance = fitness).
  double loop_noise_yaw_sigma_deg = 1.5;
  // Only with loop_planar_correction = true: the planar factor's x / y sigma
  // (metres). The fork's "variance = fitness" is a 0.25-0.39 m sigma for the
  // 0.06-0.15 fitness range a corridor loop scores, which a few hundred
  // odometry edges outvote: with yaw fixed, dp1f_1008_2's east corridor still
  // closed only to 0.4-0.5 m (0.1 m: 0.08-0.11 m; 0.05 m: 0.01-0.04 m).
  // <= 0 = variance = fitness.
  double loop_noise_xy_sigma_m = 0.05;
  // Per-keyframe tilt anchor (GravityPriorFactor): each keyframe's roll /
  // pitch w.r.t. gravity held to the LIO's value with this sigma (degrees).
  // Required once loops correct yaw with real weight -- without it the
  // optimiser realises a yaw correction partly as pitch and the chain
  // climbs or sinks (5-17 cm per loop on the 2026-10-09 replays). <= 0 = no
  // anchor, the behaviour before 2026-10-09.
  double keyframe_tilt_sigma_deg = 0.5;
  int loop_submap_half_range = 5;
  double submap_resolution = 0.1;
  double min_loop_detect_duration = 10.0;
};

class SimplePGO
{
public:
  SimplePGO(const Config & config);

  bool isKeyPose(const PoseWithTime & pose);

  bool addKeyPose(const CloudWithPose & cloud_with_pose);

  bool hasLoop() { return m_cache_pairs.size() > 0; }
  // Loops accepted by the last searchForLoopPairs() and not yet folded into
  // the graph. Valid only between that call and smoothAndUpdate(), which
  // consumes them; pgo_node reads it in that window to log each closure.
  const std::vector<LoopPair> & pendingLoops() const { return m_cache_pairs; }

  void searchForLoopPairs();

  void smoothAndUpdate();

  CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
  // Registers source onto target with the configured backend. Returns false
  // when the backend did not converge; fitness is pcl::Registration's
  // getFitnessScore() (mean squared nearest-neighbour distance over all source
  // points) for both backends, so loop_score_tresh means the same thing
  // whichever is selected.
  bool alignLoop(
    const CloudType::Ptr & source, const CloudType::Ptr & target, M4F & transform,
    double & fitness);
  std::vector<std::pair<size_t, size_t>> & historyPairs() { return m_history_pairs; }
  std::vector<KeyPoseWithCloud> & keyPoses() { return m_key_poses; }

  M3D offsetR() { return m_r_offset; }
  V3D offsetT() { return m_t_offset; }

private:
  Config m_config;
  std::vector<KeyPoseWithCloud> m_key_poses;
  std::vector<std::pair<size_t, size_t>> m_history_pairs;
  std::vector<LoopPair> m_cache_pairs;
  M3D m_r_offset;
  V3D m_t_offset;
  std::shared_ptr<gtsam::ISAM2> m_isam2;
  gtsam::Values m_initial_values;
  gtsam::NonlinearFactorGraph m_graph;
  pcl::IterativeClosestPoint<PointType, PointType> m_icp;
  small_gicp::RegistrationPCL<PointType, PointType> m_gicp;
};
