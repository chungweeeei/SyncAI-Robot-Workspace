// ApproachLaw, ROS-free: single commands, and the closed loop against a
// kinematic robot that executes each command after a delay -- the gait's
// latency, which is what makes floored axes hunt across a band without
// hysteresis (SyncAI-Robot-Charger measured ~0.3 s from command to legs and
// ~0.5 s to the reading).

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <fstream>
#include <map>
#include <string>

#include "syncai_controller/plugins/approach_controller/approach_law.hpp"
#include "syncai_controller/plugins/approach_goal_check.hpp"

using syncai_controller::ApproachGoalCheck;
using syncai_controller::ApproachGoalCheckParams;
using syncai_controller::ApproachLaw;
using syncai_controller::ApproachLawParams;
using syncai_controller::Pose2;
using syncai_controller::targetErrors;
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

// Arrived within `xy` and `yaw` of the target. The defaults are where the
// default bands let the axes rest (hypot(0.02, 0.03) and 0.08 rad) plus what
// the floors carry through the delay.
void expectAtTarget(const LoopResult & run, double xy = 0.04, double yaw = 0.1)
{
  const Pose2 target = chargerTarget();
  ASSERT_TRUE(run.arrived) << "still moving at " << run.robot.x << ", " << run.robot.y << ", "
                           << run.robot.yaw;
  EXPECT_LT(std::hypot(run.robot.x - target.x, run.robot.y - target.y), xy);
  EXPECT_LT(std::fabs(normalize(run.robot.yaw - target.yaw)), yaw);
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

  // Widened to floor x 0.6 s, it settles -- coarser: the heading may now
  // rest anywhere up to 0.05 + 0.06 rad, so charge_goal_checker would have to
  // be widened with it. Measure the latency before tuning the bands.
  ApproachLawParams wide;
  wide.x_tolerance = 0.015;
  wide.x_hysteresis = 0.02;
  wide.lateral_tolerance = 0.02;
  wide.lateral_hysteresis = 0.04;
  wide.gate_lateral_min = 0.07;
  wide.heading_tolerance = 0.05;
  wide.heading_hysteresis = 0.06;
  ASSERT_EQ(wide.validate(), "");
  expectAtTarget(
    simulate(start, 0.6, 90.0, wide), 0.05, wide.heading_tolerance + wide.heading_hysteresis);
}

TEST(ApproachLaw, TheStrafeFloorIsMetInTheBodyFrame)
{
  // 0.7 m out and 0.3 m off the axis, nose on the tag: the body is turned
  // ~0.29 rad off the dock's axis. The strafe along that axis at the floor
  // was 0.10 * cos(0.29) = 0.096 of body vy -- under the gait's floor, a
  // strafe that never starts. The floor is the gait's, so it is owed in the
  // body frame.
  ApproachLaw law(ApproachLawParams{});
  ApproachLaw::Mode mode;
  const Pose2 robot{1.0, -0.3, M_PI - std::atan2(0.3, 1.0)};
  const Velocity2 v = law.compute(robot, chargerTarget(), &mode, nullptr);
  EXPECT_EQ(mode, ApproachLaw::Mode::Align);
  EXPECT_GE(std::fabs(v.vy), ApproachLawParams{}.v_lateral_min - 1e-9);
  EXPECT_LE(std::fabs(v.vy), ApproachLawParams{}.v_lateral_max + 1e-9);

  // On the axis nothing is raised.
  ApproachLaw straight(ApproachLawParams{});
  const Velocity2 on_axis =
    straight.compute(Pose2{0.3 + 0.05, 0.0, M_PI}, chargerTarget(), &mode, nullptr);
  EXPECT_EQ(mode, ApproachLaw::Mode::Approach);
  EXPECT_NEAR(on_axis.vx, ApproachLawParams{}.v_linear_min, 1e-9);
}

TEST(ApproachLaw, TheHeadingReferenceIsHandedOverOnce)
{
  // Rocking across final_heading_distance (0.10 m): on the way in the
  // reference goes to the target's yaw at 0.10, and comes back to the view
  // point only beyond 0.15.
  ApproachLaw law(ApproachLawParams{});
  ApproachLaw::Errors e;
  const auto at = [&](double along) {
    law.compute(Pose2{0.3 + along, 0.03, M_PI}, chargerTarget(), nullptr, &e);
    return e.on_view_point;
  };
  EXPECT_TRUE(at(0.20));
  EXPECT_TRUE(at(0.11));
  EXPECT_FALSE(at(0.09));
  EXPECT_FALSE(at(0.11));
  EXPECT_FALSE(at(0.14));
  EXPECT_TRUE(at(0.16));
  EXPECT_TRUE(at(0.11));

  // reset() forgets it: a new goal starts on the view point.
  EXPECT_FALSE(at(0.05));
  law.reset();
  EXPECT_TRUE(at(0.12));
}

TEST(ApproachGoalCheck, AcceptsTheLawAtRestAndNotBefore)
{
  // The closed loop of FromOffTheAxisAndTurnedAway, with the goal check run
  // beside it as track_pose runs it: it must accept once the law holds, and
  // not while the robot is still on its way.
  ApproachLaw law(ApproachLawParams{});
  ApproachGoalCheck check{ApproachGoalCheckParams{}};
  const Pose2 target = chargerTarget();
  Pose2 robot{1.3, 0.15, M_PI - 0.4};
  std::deque<Velocity2> pipeline(6);  // 0.3 s
  double first_hold = -1.0;
  double reached = -1.0;
  for (double t = 0.0; t < 60.0 && reached < 0.0; t += kDt) {
    ApproachLaw::Mode mode;
    pipeline.push_back(law.compute(robot, target, &mode, nullptr));
    const Velocity2 v = pipeline.front();
    pipeline.pop_front();
    if (mode == ApproachLaw::Mode::Hold && first_hold < 0.0) {
      first_hold = t;
    }
    if (check.update(targetErrors(robot, target), t)) {
      reached = t;
    }
    const double c = std::cos(robot.yaw);
    const double s = std::sin(robot.yaw);
    robot.x += (c * v.vx - s * v.vy) * kDt;
    robot.y += (s * v.vx + c * v.vy) * kDt;
    robot.yaw = normalize(robot.yaw + v.wz * kDt);
  }
  ASSERT_GT(reached, 0.0) << "never accepted; robot at " << robot.x << ", " << robot.y << ", "
                          << robot.yaw;
  // Never before the robot has been inside for the settling time, so never
  // on entering the tolerances.
  const auto e = targetErrors(robot, target);
  EXPECT_LE(std::fabs(e.along), ApproachGoalCheckParams{}.along_tolerance);
  EXPECT_LE(std::fabs(e.lateral), ApproachGoalCheckParams{}.lateral_tolerance);
  EXPECT_LE(std::fabs(e.yaw), ApproachGoalCheckParams{}.yaw_tolerance);
  ASSERT_GT(first_hold, 0.0);
}

TEST(ApproachGoalCheck, ARobotCrossingTheTargetIsNotThere)
{
  // Straight across the axis at the strafe's floor, 0.10 m/s: 0.7 s inside
  // the lateral tolerance, under settle_time. dock_goal_checker's latch took
  // this robot the moment it came within 5 cm.
  ApproachGoalCheck check{ApproachGoalCheckParams{}};
  const Pose2 target = chargerTarget();
  bool reached = false;
  for (double t = 0.0; t < 3.0; t += kDt) {
    const Pose2 robot{0.3, -0.15 + 0.10 * t, M_PI};
    reached = reached || check.update(targetErrors(robot, target), t);
  }
  EXPECT_FALSE(reached);

  // In, out and in again starts the settling over.
  check.reset();
  const Pose2 on{0.3, 0.0, M_PI};
  const Pose2 off{0.3, 0.1, M_PI};
  EXPECT_FALSE(check.update(targetErrors(on, target), 0.0));
  EXPECT_FALSE(check.update(targetErrors(on, target), 0.9));
  EXPECT_FALSE(check.update(targetErrors(off, target), 0.95));
  EXPECT_FALSE(check.update(targetErrors(on, target), 1.0));
  EXPECT_FALSE(check.update(targetErrors(on, target), 1.9));
  EXPECT_TRUE(check.update(targetErrors(on, target), 2.0));
}

TEST(ApproachGoalCheck, TargetErrorsAreInTheTargetsFrame)
{
  // The charger target faces -x (yaw pi). A robot 0.2 m short of it stands at
  // x 0.5; its left, facing -x, is -y.
  const auto e = targetErrors(Pose2{0.5, -0.05, M_PI - 0.1}, chargerTarget());
  EXPECT_NEAR(e.along, 0.2, 1e-9);
  EXPECT_NEAR(e.lateral, 0.05, 1e-9);
  EXPECT_NEAR(e.yaw, 0.1, 1e-9);
}

namespace
{

// The doubles of one plugin block ("    <name>:" under ros__parameters) of the
// controller's params file: enough YAML for "      key: 0.01" lines.
std::map<std::string, double> readBlock(const std::string & file, const std::string & name)
{
  std::map<std::string, double> values;
  std::ifstream in(file);
  std::string line;
  bool inside = false;
  while (std::getline(in, line)) {
    const std::size_t indent = line.find_first_not_of(' ');
    if (indent == std::string::npos || line[indent] == '#') {
      continue;
    }
    if (!inside) {
      inside = indent == 4 && line.compare(indent, name.size() + 1, name + ":") == 0;
      continue;
    }
    if (indent <= 4) {
      break;
    }
    const std::size_t colon = line.find(':', indent);
    if (indent != 6 || colon == std::string::npos) {
      continue;
    }
    try {
      values[line.substr(indent, colon - indent)] = std::stod(line.substr(colon + 1));
    } catch (const std::exception &) {
      // Not a number (plugin: "..."), not ours.
    }
  }
  return values;
}

}  // namespace

TEST(ApproachGoalCheck, TheParamsFileKeepsTheCheckerWiderThanTheLawsRest)
{
  // The rule no code can check at run time (two plugins): where the law lets
  // an axis rest -- its tolerance + hysteresis -- must be inside what the
  // charge goal checker accepts, or the robot stands still short of success
  // until max_duration.
  const auto law = readBlock(CONTROLLER_PARAMS_FILE, "Charging");
  const auto check = readBlock(CONTROLLER_PARAMS_FILE, "charge_goal_checker");
  ASSERT_TRUE(law.count("x_tolerance") && law.count("x_hysteresis"));
  ASSERT_TRUE(law.count("lateral_tolerance") && law.count("lateral_hysteresis"));
  ASSERT_TRUE(law.count("heading_tolerance") && law.count("heading_hysteresis"));
  ASSERT_TRUE(check.count("along_tolerance") && check.count("lateral_tolerance"));
  ASSERT_TRUE(check.count("yaw_tolerance"));
  EXPECT_GT(check.at("along_tolerance"), law.at("x_tolerance") + law.at("x_hysteresis"));
  EXPECT_GT(
    check.at("lateral_tolerance"), law.at("lateral_tolerance") + law.at("lateral_hysteresis"));
  EXPECT_GT(check.at("yaw_tolerance"), law.at("heading_tolerance") + law.at("heading_hysteresis"));
}
