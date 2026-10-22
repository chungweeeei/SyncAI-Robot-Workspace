#pragma once
#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "syncai_localizer/localizers/commons.h"

// Localizability of one scan-to-map registration, and the projection of its
// correction onto the directions the scan actually constrains. New code
// (2026-10), not upstream-derived; in the global namespace like
// global_localizer.* so the localizers/ headers stay uniform (commons.h).
//
// The problem it solves. GICP against the map reports "converged" whenever the
// Gauss-Newton step got small, which says nothing about whether the solution
// was *determined*: in a long corridor every scan point lies on the two walls,
// nothing pins the pose along the corridor axis, and the optimum slides along
// it to wherever the sparse MID360 sampling happens to favour this round. That
// solution still passes every gate the node has (hasConverged, the fitness
// bound, the inlier count), and the node used to take it whole -- the
// correction along the corridor then came from GICP's noise instead of from
// Point-LIO, which knows that direction to centimetres. The same mechanism is
// behind the two-solution bounce in the west-end corner of 2026-08-28
// (params/localizer_params.yaml, rough_registration_type): y was weakly
// constrained there and the refine solution landed in either basin.
//
// The information GICP already has about this is its final 6x6 Hessian
// H = sum_i J_i^T M_i J_i (small_gicp::RegistrationResult::H): the quadratic
// cost of moving the pose by a small delta is delta^T H delta, so an
// eigenvector of H with a small eigenvalue is a direction the scan does not
// constrain. This is the "localizability" analysis of X-ICP (Tuna et al.,
// T-RO 2024) in its simplest form -- their full/partial/none classification per
// principal direction, with the re-solve of partial directions replaced by a
// damping gain -- and the degeneracy test of Zhang & Singh (ICRA 2016) before
// it. A single condition number is deliberately NOT used: PALoc's author notes
// that eigenvalues lose their axis order after a decomposition and a scalar
// threshold then misreads which direction is weak. Here each direction is
// classified on its own eigenvalue.
//
// Frames. small_gicp optimises T_target_source = T * se3_exp(delta) with
// delta = [omega(3); v(3)] (registration/optimizer.hpp), i.e. a right
// perturbation in the SOURCE (body) frame: the GICP factor's Jacobian is
// J = [R * skew(p), -R] (factors/gicp_factor.hpp), so H.block<3,3>(3,3) is the
// translation information in body coordinates. A body perturbation
// [omega_b; v_b] is the same physical motion as the map-frame (left)
// perturbation [R omega_b; R v_b], so the information in map coordinates is
// Lambda = blockdiag(R, R) H blockdiag(R, R)^T. The node works in the map frame
// (the correction it writes is map -> odom), so everything below is expressed
// there and the weak direction of a corridor is a fixed map direction whatever
// the robot's heading.
//
// Coupling. Yaw and lateral translation are coupled at range (rotating about
// the body by 1 deg moves a wall 20 m away by 0.35 m sideways), so the plain
// translation block overstates how well the lateral direction is pinned when
// yaw is itself free. The analysis therefore uses the marginal information of
// each block, the Schur complement Lambda_tt - Lambda_tr Lambda_rr^+ Lambda_rt
// (and the mirror for rotation), with a pseudo-inverse so a block that is
// itself singular does not blow the other one up.
//
// Scale. H grows with the number of inliers and with the inverse covariances
// small_gicp regularises its point distributions with (1e-3 along a plane
// normal), so the absolute values depend on the scan's density and the map's
// surface quality. The eigenvalues are divided by num_inliers and classified
// mostly by their RATIO to the largest one, which is scale-free; the optional
// absolute floor (min_eig_per_inlier) exists for the case where every
// direction is weak at once (few inliers, open space) and the ratio alone
// would still call the least-weak one FULL.
//
// What the node does with it. For each direction: FULL takes GICP's
// correction as it is, PARTIAL takes partial_gain of it, NONE takes none of
// it -- the pose along a NONE direction is left where odometry predicted it.
// This is a projection of the *update* (GICP result minus the odometry
// prediction), never a rejection of the round: a corridor round still corrects
// the two directions it can see. All three translation directions NONE means
// the scan constrained nothing and the round is odometry-only.
//
// The thresholds (LocalizabilityConfig) are first guesses. The node logs the
// eigenvalues of every accepted round (`loc: t-eig=...`), and the values are to
// be set from the dp1f replay's corridor stretches and the west-end corner --
// see the README. enabled=false keeps the analysis and the log but makes the
// projection the identity, i.e. the node's behaviour before 2026-10.

struct LocalizabilityConfig
{
  // false: analyse and log, but project nothing (the pre-2026-10 behaviour).
  bool enabled = true;
  // A direction whose per-inlier eigenvalue is below ratio_none x the largest
  // one is NONE; below ratio_partial x it, PARTIAL; otherwise FULL.
  double ratio_none = 0.02;
  double ratio_partial = 0.1;
  // Absolute floor on the per-inlier eigenvalue, in H's own units (m^-2 for
  // translation, rad^-2 for rotation, both per inlier). 0 disables it. For
  // the all-directions-weak case that the ratio test cannot see.
  double min_eig_per_inlier = 0.0;
  // Fraction of GICP's correction taken along a PARTIAL direction. X-ICP
  // re-solves those directions with the correspondences that contribute to
  // them; a damping gain is the cheap stand-in for that.
  double partial_gain = 0.3;
};

enum class Localizable : uint8_t { kFull, kPartial, kNone };

struct Localizability
{
  // Per block: eigenvalues ascending, per inlier, of the marginal information
  // in the MAP frame; the matching eigenvectors are the columns of *_vec.
  V3D trans_eig = V3D::Zero();
  M3D trans_vec = M3D::Identity();
  V3D rot_eig = V3D::Zero();
  M3D rot_vec = M3D::Identity();
  std::array<Localizable, 3> trans_level{
    Localizable::kNone, Localizable::kNone, Localizable::kNone};
  std::array<Localizable, 3> rot_level{Localizable::kNone, Localizable::kNone, Localizable::kNone};
  // false when there was nothing to analyse (no inliers, a zero or non-finite
  // H); the levels are then all NONE and the projections are the identity.
  bool valid = false;

  bool transFullyDegenerate() const;
  // "FFN/FFF": translation levels then rotation levels, weakest first.
  std::string levelString() const;
  // Heading (map frame, rad) of the weakest translation direction, for a log
  // line; the eigenvector's sign is arbitrary, so this is folded to [0, pi).
  double weakestTransHeading() const;
};

// H_body: small_gicp's final Hessian (body-frame right perturbation,
// [omega; v]). R_map_body: the rotation of the registered pose. num_inliers:
// small_gicp's inlier count for the same stage.
Localizability analyzeLocalizability(
  const Eigen::Matrix<double, 6, 6> & H_body, const M3D & R_map_body, std::size_t num_inliers,
  const LocalizabilityConfig & cfg);

// Project a map-frame translation difference (GICP minus prediction) onto the
// constrained directions: FULL x1, PARTIAL x partial_gain, NONE x0. Identity
// when the analysis is invalid or the config is disabled.
V3D projectTranslation(
  const Localizability & loc, const V3D & dt_map, const LocalizabilityConfig & cfg);

// Same for a map-frame rotation vector (axis x angle, the left perturbation
// log(R_gicp R_pred^T)).
V3D projectRotation(
  const Localizability & loc, const V3D & dtheta_map, const LocalizabilityConfig & cfg);
