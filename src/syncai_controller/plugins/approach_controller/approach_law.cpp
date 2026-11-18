#include "syncai_controller/plugins/approach_controller/approach_law.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace syncai_controller
{

namespace
{

double normalize(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

std::string ApproachLawParams::validate() const
{
  const auto pair = [](const char * floor, double f, const char * cap, double c) -> std::string {
    if (f <= 0.0 || c < f) {
      return std::string(floor) + " (" + std::to_string(f) + ") must be > 0 and <= " + cap + " (" +
             std::to_string(c) + ")";
    }
    return "";
  };
  std::string why = pair("v_linear_min", v_linear_min, "v_linear_max", v_linear_max);
  if (why.empty()) {
    why = pair("v_linear_min", v_linear_min, "v_reverse_max", v_reverse_max);
  }
  if (why.empty()) {
    why = pair("v_lateral_min", v_lateral_min, "v_lateral_max", v_lateral_max);
  }
  if (why.empty()) {
    why = pair("v_angular_min", v_angular_min, "v_angular_max", v_angular_max);
  }
  if (!why.empty()) {
    return why;
  }
  if (k_x <= 0.0 || k_y <= 0.0 || k_yaw <= 0.0) {
    return "k_x, k_y and k_yaw must be > 0";
  }
  if (view_distance < 0.0 || final_heading_distance < 0.0 || final_heading_hysteresis < 0.0) {
    return "view_distance, final_heading_distance and final_heading_hysteresis must be >= 0";
  }
  if (
    x_tolerance <= 0.0 || lateral_tolerance <= 0.0 || heading_tolerance <= 0.0 ||
    x_hysteresis < 0.0 || lateral_hysteresis < 0.0 || heading_hysteresis < 0.0) {
    return "the tolerances must be > 0 and the hystereses >= 0";
  }
  for (std::size_t i = 0; i < speed_table.size(); ++i) {
    if (speed_table[i].first <= 0.0 || speed_table[i].second <= 0.0) {
      return "speed_table entries must be > 0";
    }
    if (i > 0 && speed_table[i].first >= speed_table[i - 1].first) {
      return "speed_table distances must be strictly decreasing";
    }
  }
  if (gate_reopen_ratio <= 0.0 || gate_reopen_ratio > 1.0) {
    return "gate_reopen_ratio must be in (0, 1]";
  }
  // A closed gate waits for the strafe and the turn to bring their errors
  // under its reopen limit, and an axis at rest may sit anywhere up to its
  // tolerance + hysteresis. compute() therefore never asks for less than that
  // to reopen, and the gate must close above it, or it would close on an
  // error nothing is going to correct and wait forever.
  if (gate_lateral_min <= lateral_tolerance + lateral_hysteresis) {
    return "gate_lateral_min must be > lateral_tolerance + lateral_hysteresis";
  }
  if (gate_heading <= heading_tolerance + heading_hysteresis) {
    return "gate_heading must be > heading_tolerance + heading_hysteresis";
  }
  if (gate_lateral_slope < 0.0) {
    return "gate_lateral_slope must be >= 0";
  }
  // Turning in place ends once the error is under this; it must then be
  // inside what the gate tolerates, or the walk in could never start.
  if (rotate_in_place_angle <= gate_heading) {
    return "rotate_in_place_angle must be > gate_heading";
  }
  return "";
}

ApproachLaw::ApproachLaw(const ApproachLawParams & params) : params_(params)
{
}

void ApproachLaw::setParams(const ApproachLawParams & params)
{
  params_ = params;
}

void ApproachLaw::reset()
{
  x_active_ = false;
  lateral_active_ = false;
  heading_active_ = false;
  gate_open_ = false;
  on_target_yaw_ = false;
}

bool ApproachLaw::band(bool active, double error, double tolerance, double hysteresis)
{
  const double magnitude = std::fabs(error);
  if (active) {
    return magnitude > tolerance;
  }
  return magnitude > tolerance + hysteresis;
}

double ApproachLaw::floored(double error, double gain, double floor, double cap)
{
  return std::copysign(std::clamp(gain * std::fabs(error), floor, cap), error);
}

double ApproachLaw::forwardCap(double along) const
{
  double cap = params_.v_linear_max;
  for (const auto & [distance, speed] : params_.speed_table) {
    if (along <= distance) {
      cap = speed;
    }
  }
  // A table speed under the floor would make the floor the cap anyway; say so
  // here rather than let clamp() see floor > cap.
  return std::max(cap, params_.v_linear_min);
}

Velocity2 ApproachLaw::compute(
  const Pose2 & robot, const Pose2 & target, Mode * mode, Errors * errors)
{
  const ApproachLawParams & p = params_;

  // The errors in the target's frame: along its heading and across it.
  const double c = std::cos(target.yaw);
  const double s = std::sin(target.yaw);
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  Errors e;
  e.along = c * dx + s * dy;
  e.lateral = -s * dx + c * dy;
  e.yaw = normalize(target.yaw - robot.yaw);

  // The heading to hold. Walking in, the nose stays on the view point (the tag
  // for a charger); with the robot on the target's axis that IS the target's
  // yaw, so the hand-over to the yaw at final_heading_distance only jumps by
  // atan(lateral / (along + view_distance)), small once the gate has let the
  // robot walk that far. Closer in the view point's bearing turns fast with
  // every centimetre of strafe, and the final heading is the target's anyway.
  // The hand-over has hysteresis (see final_heading_hysteresis).
  if (on_target_yaw_) {
    on_target_yaw_ = e.along <= p.final_heading_distance + p.final_heading_hysteresis;
  } else {
    on_target_yaw_ = e.along <= p.final_heading_distance;
  }
  e.heading_ref = target.yaw;
  if (p.view_distance > 0.0 && !on_target_yaw_) {
    const double vx = target.x + p.view_distance * c;
    const double vy = target.y + p.view_distance * s;
    e.heading_ref = std::atan2(vy - robot.y, vx - robot.x);
    e.on_view_point = true;
  }
  e.heading = normalize(e.heading_ref - robot.yaw);

  Velocity2 cmd;
  Mode m = Mode::Hold;

  if (std::fabs(e.heading) > p.rotate_in_place_angle) {
    // Turn first, nothing else: strafing with the nose far off the view point
    // is how the tag leaves the image.
    heading_active_ = true;
    gate_open_ = false;
    cmd.wz = floored(e.heading, p.k_yaw, p.v_angular_min, p.v_angular_max);
    m = Mode::RotateToView;
  } else {
    heading_active_ = band(heading_active_, e.heading, p.heading_tolerance, p.heading_hysteresis);
    lateral_active_ = band(lateral_active_, e.lateral, p.lateral_tolerance, p.lateral_hysteresis);
    x_active_ = band(x_active_, e.along, p.x_tolerance, p.x_hysteresis);

    const double gate_lateral = p.gate_lateral_min + p.gate_lateral_slope * std::max(e.along, 0.0);
    if (gate_open_) {
      gate_open_ = std::fabs(e.lateral) <= gate_lateral && std::fabs(e.heading) <= p.gate_heading;
    } else {
      // Never below where an axis at rest may sit (see validate()).
      const double reopen_lateral =
        std::max(p.gate_reopen_ratio * gate_lateral, p.lateral_tolerance + p.lateral_hysteresis);
      const double reopen_heading =
        std::max(p.gate_reopen_ratio * p.gate_heading, p.heading_tolerance + p.heading_hysteresis);
      gate_open_ = std::fabs(e.lateral) <= reopen_lateral && std::fabs(e.heading) <= reopen_heading;
    }

    // Target frame -> body frame: rotate by -(robot yaw - target yaw). With
    // the body turned to keep the tag in view, a pure strafe along the
    // target's axis is a body command with some forward in it, and the other
    // way round (the aligner did the same with its Rz).
    const double delta = normalize(robot.yaw - target.yaw);
    const double cd = std::cos(delta);
    const double sd = std::sin(delta);
    // The gait's floors are body-frame: each axis's floor is raised so its
    // main body component still meets it (see ApproachLawParams). A cap under
    // the raised floor gives way to it, as forwardCap() does for the table.
    const double raise = 1.0 / std::cos(std::min(std::fabs(delta), kMaxFloorDelta));
    const double linear_floor = p.v_linear_min * raise;
    const double lateral_floor = p.v_lateral_min * raise;

    double v_along = 0.0;
    if (x_active_ && e.along < 0.0) {
      // Past the target: back up whatever the gate says. Backing away from
      // the dock is the safe direction, and the tag only grows in the image.
      v_along = floored(e.along, p.k_x, linear_floor, std::max(p.v_reverse_max, linear_floor));
      m = Mode::Reverse;
    } else if (x_active_ && gate_open_) {
      v_along = floored(e.along, p.k_x, linear_floor, std::max(forwardCap(e.along), linear_floor));
      m = Mode::Approach;
    } else if (x_active_ || lateral_active_ || heading_active_) {
      m = Mode::Align;
    }
    const double v_lateral =
      lateral_active_
        ? floored(e.lateral, p.k_y, lateral_floor, std::max(p.v_lateral_max, lateral_floor))
        : 0.0;
    if (heading_active_) {
      cmd.wz = floored(e.heading, p.k_yaw, p.v_angular_min, p.v_angular_max);
    }

    cmd.vx = cd * v_along + sd * v_lateral;
    cmd.vy = -sd * v_along + cd * v_lateral;
  }

  e.gate_open = gate_open_;
  if (mode) {
    *mode = m;
  }
  if (errors) {
    *errors = e;
  }
  return cmd;
}

const char * toString(ApproachLaw::Mode mode)
{
  switch (mode) {
    case ApproachLaw::Mode::RotateToView:
      return "rotate to view";
    case ApproachLaw::Mode::Align:
      return "align";
    case ApproachLaw::Mode::Approach:
      return "approach";
    case ApproachLaw::Mode::Reverse:
      return "reverse";
    case ApproachLaw::Mode::Hold:
      return "hold";
  }
  return "unknown";
}

}  // namespace syncai_controller
