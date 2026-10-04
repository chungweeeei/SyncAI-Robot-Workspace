#pragma once
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <cmath>
#include <functional>

/**
 * A loop-closure factor that constrains only what a loop is trusted for:
 * the WORLD-frame x / y offset and the yaw difference between the target
 * (old) and source (new) keyframes. z, roll and pitch are not in the error
 * at all, so the loop cannot ask for them and the optimiser has no reason
 * to bend the chain in pitch to satisfy it.
 *
 * Why not a BetweenFactor<Pose3> with a tight roll / pitch / z noise: that
 * factor's error lives in the measured relative pose's frame, i.e. the
 * lidar body frame, which on this robot is pitched ~15 deg. Its x axis
 * therefore carries sin(15 deg) ~ 0.26 of world z, and every centimetre of
 * x correction the loop asked for leaked a quarter of it into z. The 2026-10
 * dp1f_1002 replay measured that leak as a flat -2..-4 cm offset after the
 * first loop and -9 cm at the last one (see the package README).
 *
 * Error (3-dim): [ (t_s - t_t).xy - m.xy ;  wrap(yaw(s) - yaw(t) - m.yaw) ],
 * with yaw = atan2(R10, R00) of each pose's rotation. Jacobians are
 * numerical (gtsam::numericalDerivative): a handful of loops per run, so
 * the cost is irrelevant and there is no analytic expression to get wrong.
 */
class PlanarLoopFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  // meas = (dx, dy, dyaw): source relative to target, in the world frame.
  PlanarLoopFactor(
    gtsam::Key target, gtsam::Key source, const gtsam::Vector3 & meas,
    const gtsam::SharedNoiseModel & model)
  : Base(model, target, source), m_meas(meas)
  {
  }

  static double yawOf(const gtsam::Pose3 & p)
  {
    const gtsam::Matrix3 R = p.rotation().matrix();
    return std::atan2(R(1, 0), R(0, 0));
  }

  static double wrapAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // What the two poses currently say, in the measurement's parametrisation.
  static gtsam::Vector3 planarDelta(const gtsam::Pose3 & target, const gtsam::Pose3 & source)
  {
    gtsam::Vector3 d;
    d.head<2>() = (source.translation() - target.translation()).head<2>();
    d(2) = wrapAngle(yawOf(source) - yawOf(target));
    return d;
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & target, const gtsam::Pose3 & source,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const override
  {
    const gtsam::Vector3 meas = m_meas;
    const std::function<gtsam::Vector3(const gtsam::Pose3 &, const gtsam::Pose3 &)> err =
      [meas](const gtsam::Pose3 & t, const gtsam::Pose3 & s) {
        gtsam::Vector3 e = planarDelta(t, s) - meas;
        e(2) = wrapAngle(e(2));
        return e;
      };
    if (H1)
      *H1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        err, target, source);
    if (H2)
      *H2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        err, target, source);
    return err(target, source);
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new PlanarLoopFactor(*this)));
  }

  const gtsam::Vector3 & measured() const { return m_meas; }

private:
  gtsam::Vector3 m_meas;
};
