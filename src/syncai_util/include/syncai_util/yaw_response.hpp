#ifndef SYNCAI_UTIL__YAW_RESPONSE_HPP_
#define SYNCAI_UTIL__YAW_RESPONSE_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace syncai_util
{

// What yaw rate the body turns at for a commanded cmd_vel angular.z
// (2026-11-27): the gait's steady response, measured, for a controller to
// predict with. ROS-free, header-only; test/test_yaw_response.cpp.
//
// Why a controller needs it. The Lynx M20's gait does not turn below a yaw
// command of ~0.5 rad/s and then turns at once: robot01's open-loop steps in
// the Basic gait (record/m20_stair_1127/t5_turn_response, standing and walking
// alike) turned the body at 0 for 0.3 / 0.5, 0.15-0.36 (left) and 0.49-0.50
// (right) for 0.6, 0.53 for 0.7, 0.93-0.95 for 1.0. A controller that predicts
// with angular.z itself plans corrections the robot does not make (a flat arc
// left 0.5 m off its plan) -- and an inverse table at the driver cannot
// straighten a response that jumps: it only moves the jump to the table's
// threshold, where a 0.19 command turned nothing and a 0.21 turned 0.33, and
// with the pose ~0.11 s late MPPI weaved +-5 deg on a straight line
// (t6_weave). Predicting with the response itself, the controller knows a
// small command does nothing and a correction costs a real turn.
//
// One table a side, (command -> rate), from command 0 / rate 0, linear between
// points and proportional past the last (rate = last_rate * |command| /
// last_command). An empty side: the identity, the controller's assumption
// before this existed.
struct YawResponseSide
{
  std::vector<double> command;
  std::vector<double> rate;
};

class YawResponse
{
public:
  YawResponse() = default;

  // Why a side cannot be used, or nullopt. Empty (both lists) is the identity.
  static std::optional<std::string> problem(const YawResponseSide & side)
  {
    if (side.command.empty() && side.rate.empty()) return std::nullopt;
    if (side.command.size() != side.rate.size() || side.command.size() < 2) {
      return std::to_string(side.command.size()) + " commands and " +
             std::to_string(side.rate.size()) + " rates: needs the same number, at least 2";
    }
    for (std::size_t i = 0; i < side.command.size(); ++i) {
      if (!std::isfinite(side.command[i]) || !std::isfinite(side.rate[i]) ||
          side.command[i] < 0.0 || side.rate[i] < 0.0)
      {
        return std::string("every value must be finite and >= 0");
      }
    }
    if (side.command[0] != 0.0 || side.rate[0] != 0.0) {
      return std::string("the first point must be command 0, rate 0");
    }
    for (std::size_t i = 1; i < side.command.size(); ++i) {
      if (side.command[i] <= side.command[i - 1]) return std::string("the commands must rise");
      if (side.rate[i] < side.rate[i - 1]) return std::string("the rates must not fall");
    }
    if (side.rate.back() <= 0.0) return std::string("the last rate must be > 0");
    return std::nullopt;
  }

  // Throws std::invalid_argument naming the side when one cannot be used.
  void set(const YawResponseSide & left, const YawResponseSide & right)
  {
    if (auto p = problem(left)) throw std::invalid_argument("yaw response, left: " + *p);
    if (auto p = problem(right)) throw std::invalid_argument("yaw response, right: " + *p);
    left_ = left;
    right_ = right;
  }

  bool active() const { return left_.command.size() >= 2 || right_.command.size() >= 2; }

  const YawResponseSide & left() const { return left_; }
  const YawResponseSide & right() const { return right_; }

  // The yaw rate the body turns at for `command` (rad/s; positive = left).
  double rate(double command) const
  {
    if (!std::isfinite(command) || command == 0.0) return command;
    const YawResponseSide & side = command > 0.0 ? left_ : right_;
    const std::size_t n = side.command.size();
    if (n < 2) return command;
    const double c = std::fabs(command);
    double r;
    if (c >= side.command[n - 1]) {
      r = side.rate[n - 1] * c / side.command[n - 1];
    } else {
      const std::size_t i = std::clamp<std::size_t>(
        static_cast<std::size_t>(
          std::upper_bound(side.command.begin(), side.command.end(), c) - side.command.begin()),
        1, n - 1);
      const double c0 = side.command[i - 1], c1 = side.command[i];
      const double r0 = side.rate[i - 1], r1 = side.rate[i];
      r = r0 + (r1 - r0) * (c - c0) / (c1 - c0);
    }
    return std::copysign(r, command);
  }

private:
  YawResponseSide left_;
  YawResponseSide right_;
};

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__YAW_RESPONSE_HPP_
