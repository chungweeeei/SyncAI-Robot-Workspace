#pragma once
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <functional>

/**
 * A unary factor that pins one keyframe's TILT (roll / pitch with respect to
 * gravity) to what the LIO front end measured, and leaves its yaw and
 * position free.
 *
 * Why it exists: the odometry chain is a series of 6-DOF BetweenFactors
 * expressed in the body frame, which on this robot is pitched ~15 deg, and
 * PlanarLoopFactor constrains only world x / y / yaw. When a loop is
 * allowed to correct yaw with real weight (loop_noise_yaw_sigma_deg), the
 * optimiser is free to realise that yaw with ANY rotation that is cheap in
 * the body frame -- and the cheapest ones carry a little pitch. A chain
 * whose heading is pitched by a fraction of a degree climbs or sinks along
 * every metre it drives. On the 2026-10-09 replays (yaw sigma 1.5 deg, no
 * tilt anchor) single loops moved keyframe z by 5-17 cm, and on dp1f_1006,
 * which was already closed, a 7 cm double floor appeared at a revisit.
 *
 * Point-LIO's world frame is gravity-aligned and its tilt is observable from
 * the IMU at every instant, so each keyframe's tilt is an absolute
 * measurement, not something a loop should be able to trade away. With
 * every keyframe anchored, the only way left to change a heading is to
 * rotate about world z, which does not touch z.
 *
 * Error (3-dim, rank 2): R^T e_z - g, where g = R_lio^T e_z is world "up"
 * seen in the body frame at insertion. A 3-vector instead of two angles so
 * there is no Euler singularity; the component along g is second order and
 * carries no information. Jacobian numerical, as in PlanarLoopFactor.
 */
class GravityPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  GravityPriorFactor(
    gtsam::Key key, const gtsam::Vector3 & up_in_body, const gtsam::SharedNoiseModel & model)
  : Base(model, key), m_up(up_in_body)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose, boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const gtsam::Vector3 up = m_up;
    const std::function<gtsam::Vector3(const gtsam::Pose3 &)> err = [up](const gtsam::Pose3 & p) {
      return gtsam::Vector3(p.rotation().matrix().transpose() * gtsam::Vector3::UnitZ() - up);
    };
    if (H) *H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(err, pose);
    return err(pose);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GravityPriorFactor(*this)));
  }

private:
  gtsam::Vector3 m_up;
};
