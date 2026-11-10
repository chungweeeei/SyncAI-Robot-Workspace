// ApproachLaw, ROS-free: single commands, and the closed loop against a
// kinematic robot that executes each command after a delay -- the gait's
// latency, which is what makes floored axes hunt across a band without
// hysteresis (SyncAI-Robot-Charger measured ~0.3 s from command to legs and
// ~0.5 s to the reading).

#include <gtest/gtest.h>

#include <cmath>
#include <deque>

#include "syncai_controller/plugins/approach_controller/approach_law.hpp"

using syncai_controller::ApproachLaw;
using syncai_controller::ApproachLawParams;
using syncai_controller::Pose2;
using syncai_controller::Velocity2;

namespace
{

constexpr double kDt = 0.05;  // controller_frequency 20 Hz

double normalize(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// A charger target: the tag at the origin, its +x out of the face along odom
// +x; the robot docks 0.3 m in front of it, facing it (yaw pi). The default
// view_distance 0.3 puts the view point on the tag.
Pose2 chargerTarget()
{
  return Pose2{0.3, 0.0, M_PI};
}

struct LoopResult
{
  Pose2 robot;
  double seconds{0.0};
  bool arrived{false};
  // Worst bearing of the tag (the origin) off the nose while not turning in
  // place.
  double worst_tag_bearing{0.0};
  int commands{0};
};

// Closed loop until every axis holds for 1 s, or `limit` seconds.
LoopResult simulate(Pose2 robot, double delay, double limit, ApproachLawParams params = {})
{
  ApproachLaw law(params);
  const Pose2 target = chargerTarget();
  std::deque<Velocity2> pipeline(static_cast<std::size_t>(std::round(delay / kDt)));
  LoopResult run;
  double held = 0.0;
  for (double t = 0.0; t < limit; t += kDt) {
    ApproachLaw::Mode mode;
    ApproachLaw::Errors errors;
    pipeline.push_back(law.compute(robot, target, &mode, &errors));
    const Velocity2 v = pipeline.front();
    pipeline.pop_front();

    if (mode != ApproachLaw::Mode::RotateToView) {
      const double bearing = normalize(std::atan2(-robot.y, -robot.x) - robot.yaw);
      run.worst_tag_bearing = std::max(run.worst_tag_bearing, std::fabs(bearing));
    }
    if (mode == ApproachLaw::Mode::Hold) {
      held += kDt;
      if (held >= 1.0) {
        run.arrived = true;
        run.seconds = t;
        break;
      }
    } else {
      held = 0.0;
      ++run.commands;
    }

    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);
    robot.x += (c * v.vx - s * v.vy) * kDt;
    robot.y += (s * v.vx + c * v.vy) * kDt;
    robot.yaw = normalize(robot.yaw + v.wz * kDt);
  }
  run.robot = robot;
  return run;
}

// Arrived within `xy` of the target and dock_goal_checker's 0.1 rad. The
// default `xy` leaves room under the checker's 0.05 m: the bands plus what
// the floors carry through the delay.
void expectAtTarget(const LoopResult & run, double xy = 0.04)
{
  const Pose2 target = chargerTarget();
  ASSERT_TRUE(run.arrived) << "still moving at " << run.robot.x << ", " << run.robot.y << ", "
                           << run.robot.yaw;
  EXPECT_LT(std::hypot(run.robot.x - target.x, run.robot.y - target.y), xy);
  EXPECT_LT(std::fabs(normalize(run.robot.yaw - target.yaw)), 0.1);
}

}  // namespace

TEST(ApproachLaw, DefaultsAreValid)
{
  EXPECT_EQ(ApproachLawParams{}.validate(), "");
}

TEST(ApproachLaw, ValidateRejectsAGateNothingCanReopen)
{
  ApproachLawParams p;
  p.gate_lateral_min = p.lateral_tolerance + p.lateral_hysteresis;
  EXPECT_NE(p.validate(), "");
  p = ApproachLawParams{};
  p.speed_table = {{0.1, 0.1}, {0.2, 0.05}};
  EXPECT_NE(p.validate(), "");
  p = ApproachLawParams{};
  p.v_lateral_min = 0.3;
  EXPECT_NE(p.validate(), "");
}

TEST(ApproachLaw, AtTheTargetItHolds)
{
  ApproachLaw law(ApproachLawParams{});
  ApproachLaw::Mode mode;
  const Velocity2 v = law.compute(chargerTarget(), chargerTarget(), &mode, nullptr);
  EXPECT_EQ(mode, ApproachLaw::Mode::Hold);
  EXPECT_EQ(v.vx, 0.0);
  EXPECT_EQ(v.vy, 0.0);
  EXPECT_EQ(v.wz, 0.0);
}

TEST(ApproachLaw, FarOffTheViewPointItOnlyTurns)
{
  ApproachLaw law(ApproachLawParams{});
  ApproachLaw::Mode mode;
  // 1 m out on the axis, nose 90 deg off the tag.
  const Velocity2 v = law.compute(Pose2{1.3, 0.0, M_PI / 2.0}, chargerTarget(), &mode, nullptr);
  EXPECT_EQ(mode, ApproachLaw::Mode::RotateToView);
  EXPECT_EQ(v.vx, 0.0);
  EXPECT_EQ(v.vy, 0.0);
  EXPECT_GT(v.wz, 0.0);  // the tag is on the left: turn left
}

TEST(ApproachLaw, OutsideTheGateItStrafesWithoutWalkingIn)
{
  ApproachLaw law(ApproachLawParams{});
  ApproachLaw::Mode mode;
  ApproachLaw::Errors e;
  // 1 m out, 0.3 m to the side (gate at 1 m: 0.04 + 0.15 * 0.7 = 0.145),
  // nose on the tag.
  const Pose2 robot{1.3, -0.3, M_PI - std::atan2(0.3, 1.3)};
  const Velocity2 v = law.compute(robot, chargerTarget(), &mode, &e);
  EXPECT_EQ(mode, ApproachLaw::Mode::Align);
  EXPECT_FALSE(e.gate_open);
  // In odom: no motion along the dock's axis (x), all of it toward the axis
  // (+y, the robot is at y -0.3).
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  const double odom_vx = c * v.vx - s * v.vy;
  const double odom_vy = s * v.vx + c * v.vy;
  EXPECT_NEAR(odom_vx, 0.0, 1e-9);
  EXPECT_GT(odom_vy, 0.0);
}

TEST(ApproachLaw, StraightInFromOneMetre)
{
  const LoopResult run = simulate(Pose2{1.3, 0.0, M_PI}, 0.3, 60.0);
  expectAtTarget(run);
}

TEST(ApproachLaw, FromOffTheAxisAndTurnedAway)
{
  // The staging pose's usual error: 15 cm off the axis, 0.4 rad off.
  const LoopResult run = simulate(Pose2{1.3, 0.15, M_PI - 0.4}, 0.3, 60.0);
  expectAtTarget(run);
  // Once turned onto the tag, the strafe and the walk in never take the nose
  // further off it than where turning in place stopped: the tag stays as
  // far inside the image as the turn left it.
  EXPECT_LT(run.worst_tag_bearing, ApproachLawParams{}.rotate_in_place_angle + 0.02);
}

TEST(ApproachLaw, FromBehindTheTargetItBacksUp)
{
  const LoopResult run = simulate(Pose2{0.22, 0.0, M_PI}, 0.3, 30.0);
  expectAtTarget(run);
}

TEST(ApproachLaw, ASlowerGaitNeedsBandsAsWideAsItsOvershoot)
{
  // The rule the defaults are sized by: an axis settles only if its
  // tolerance + hysteresis is wider than floor x latency, what the gait
  // carries on after the stop. The defaults are for ~0.3 s (the charger's
  // command-to-legs). At twice that the strafe's 0.10 m/s floor carries 6 cm
  // through a 3 cm band, and the robot hunts across the axis for good.
  const Pose2 start{1.0, -0.1, M_PI + 0.2};
  EXPECT_FALSE(simulate(start, 0.6, 90.0).arrived);

  // Widened to floor x 0.6 s, it settles -- coarser, still inside
  // dock_goal_checker's 0.05 m. Measure the latency before tuning the bands.
  ApproachLawParams wide;
  wide.x_tolerance = 0.015;
  wide.x_hysteresis = 0.02;
  wide.lateral_tolerance = 0.02;
  wide.lateral_hysteresis = 0.04;
  wide.gate_lateral_min = 0.07;
  wide.heading_tolerance = 0.05;
  wide.heading_hysteresis = 0.06;
  ASSERT_EQ(wide.validate(), "");
  expectAtTarget(simulate(start, 0.6, 90.0, wide), 0.05);
}
