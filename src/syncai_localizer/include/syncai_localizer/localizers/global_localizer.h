#pragma once
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "syncai_localizer/localizers/commons.h"
#include "syncai_localizer/localizers/icp_localizer.h"

// Global (hint-free) search behind the `auto_localize` service: the recovery
// for a kidnapped robot, where ICPLocalizer's local rough + refine has nothing
// to start from. Unlike icp_localizer.* this is not upstream-derived code.
//
// The search space is the map's own mapping trajectory (map/<name>/poses.txt):
// the keyframe positions are where the robot has physically stood, so they are
// walkable by construction and carry the right z, which a grid over the map's
// bounding box would have to guess. Each thinned position is tried at every
// yaw step; roll / pitch are not searched at all, because Point-LIO's odom
// frame and the map are both gravity-aligned -- the caller hands in a scan
// already rotated into a gravity-level, yaw-zero frame (the same reasoning as
// LocalizerNode::applyPlanarGuess: the ~15 deg mount pitch must be kept, or
// no candidate ever scores).
//
// Two stages, because a GICP per candidate does not fit any sane deadline
// (thousands of candidates x 30-50 ms):
//   1. coarse: transform a few hundred scan points by each candidate and count
//      how many land within coarse_inlier_dist of the rough map (one KD-tree
//      query per point, parallel over candidates);
//   2. fine: the top_k distinct coarse candidates each get a rough GICP and a
//      refine GICP, and are scored by their inlier ratio at refine resolution.
// The winner must clear min_inlier_ratio, and no *distinct* runner-up may come
// within ambiguity_ratio of it -- a symmetric corridor fails instead of
// guessing, because a confident wrong pose is the one outcome worse than no
// pose.
//
// Owns its own RegistrationPCL pair and KD-trees, never ICPLocalizer's: those
// are in use on the timer thread, and holding ICPLocalizer::m_target_mutex
// for a multi-second search would stall the TF rebroadcast. The map clouds are
// shared by pointer, which is safe because ICPLocalizer::loadMap swaps in new
// clouds and never mutates a loaded one.

struct GlobalLocConfig
{
  // How many recent body clouds the node accumulates (in the latest scan's
  // frame, through the odom poses) for one search. A single body_cloud is
  // filter_num 6 over 0.1 s of a non-repetitive MID360 scan -- too sparse to
  // tell places apart; ten of them is one second of coverage.
  int accum_scans = 10;
  // Greedy thinning of the poses.txt keyframes: a keyframe closer than this to
  // one already kept is dropped. Also the worst-case position error a coarse
  // candidate starts the fine stage with (half of it, on the diagonal ~0.7x).
  double candidate_spacing = 1.0;
  // Yaw sweep step per position. The fine stage's rough GICP has to absorb
  // half of it.
  double yaw_step_deg = 15.0;
  // Coarse stage: scan voxel size and a cap on its points (a deterministic
  // stride, so a replay reproduces), and the inlier distance against the 0.25 m
  // rough map. Loose on purpose -- the candidate is up to half a grid step and
  // half a yaw step off.
  double coarse_scan_resolution = 0.5;
  int coarse_max_points = 400;
  double coarse_inlier_dist = 1.0;
  // Fine stage: how many distinct coarse candidates get GICP.
  int top_k = 10;
  // The fine stage's rough GICP starts much further off than the timer's
  // (which starts from last round's answer), so it gets a wider correspondence
  // radius and more iterations than ICPConfig's rough_*. Its hasConverged()
  // is NOT a gate: the inlier ratio below is, and the timer re-verifies the
  // winner with its own converged-gated registration before the service
  // answers success.
  double rough_max_corr_dist = 3.0;
  int max_iteration = 50;
  // Inlier distance at refine resolution (0.1 m map) for the final score.
  double refine_inlier_dist = 0.3;
  // Acceptance. ICPConfig's *_score_thresh are sanity bounds that reject
  // nothing (see params/localizer_params.yaml), so the inlier ratio is the real
  // gate here. Both values are first guesses, not measured.
  double min_inlier_ratio = 0.6;
  double ambiguity_ratio = 0.9;
  // A runner-up is "distinct" from the winner when it is further than either
  // of these. Also the non-max suppression radius for picking top_k (scaled
  // up to at least one grid / yaw step, see the definition).
  double ambiguity_trans = 1.0;
  double ambiguity_yaw_deg = 10.0;
  // Deadlines; <= 0 disables one. search bounds GlobalLocalizer::search();
  // converge bounds the node's wait for the timer to register against the
  // winner afterwards.
  double search_timeout_sec = 10.0;
  double converge_timeout_sec = 5.0;
};

struct GlobalLocResult
{
  bool success = false;
  std::string message;
  // map_T_leveled: the pose of the gravity-level, yaw-zero scan frame the
  // caller passed in. Valid whenever a winner was picked (also on an
  // ambiguity or inlier-ratio failure, for the log / response).
  M4F pose = M4F::Identity();
  bool has_pose = false;
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  int candidates = 0;
};

class GlobalLocalizer
{
public:
  using Clock = std::chrono::steady_clock;

  GlobalLocalizer(const GlobalLocConfig & config, const ICPConfig & icp_config);

  // Keyframe positions (map frame) from a poses.txt written by
  // syncai_mapping's save_maps: "<i>.pcd x y z qw qx qy qz" per line. Thinned
  // to candidate_spacing. False with `error` set when the file is missing,
  // unreadable or yields no position.
  bool loadCandidatePositions(
    const std::string & poses_path, std::vector<V3D> & positions, std::string & error) const;

  // scan: gravity-level, yaw-zero frame, origin at the body. The maps are
  // ICPLocalizer::roughMap() / refineMap(). Rebuilds its trees / GICP targets
  // only when a map pointer differs from the previous call's, so only the
  // first call after a map load pays for them (that build is not
  // interruptible and counts against the deadline).
  GlobalLocResult search(
    const CloudType::Ptr & scan, const std::vector<V3D> & positions,
    const CloudType::Ptr & rough_map, const CloudType::Ptr & refine_map, Clock::time_point deadline);

  const GlobalLocConfig & config() const { return m_config; }

private:
  struct Tree;

  void setTargets(const CloudType::Ptr & rough_map, const CloudType::Ptr & refine_map);

  GlobalLocConfig m_config;
  ICPConfig m_icp_config;
  small_gicp::RegistrationPCL<PointType, PointType> m_rough_icp;
  small_gicp::RegistrationPCL<PointType, PointType> m_refine_icp;
  CloudType::Ptr m_rough_map;
  CloudType::Ptr m_refine_map;
  std::shared_ptr<Tree> m_rough_tree;
  std::shared_ptr<Tree> m_refine_tree;
};
