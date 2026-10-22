#include "syncai_util/stair_mode.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "syncai_util/geometry_utils.hpp"
#include "syncai_util/node_utils.hpp"
#include "syncai_util/path_height.hpp"

namespace syncai_util
{

using syncai_util::geometry_utils::euclidean_distance;

void StairMode::initialize(const rclcpp::Node::SharedPtr & node, const std::string & plugin_name)
{
  logger_ = node->get_logger();

  // The windows were set on dp1f_1012's Octo3D paths; the Params struct has
  // the numbers and why.
  declare_parameter_if_not_declared(
    node, plugin_name + ".use_stair_mode", rclcpp::ParameterValue(params_.enabled));
  declare_parameter_if_not_declared(
    node, plugin_name + ".stair_min_rise", rclcpp::ParameterValue(params_.min_rise));
  declare_parameter_if_not_declared(
    node, plugin_name + ".stair_enter_dist", rclcpp::ParameterValue(params_.enter_dist));
  declare_parameter_if_not_declared(
    node, plugin_name + ".stair_exit_behind_dist",
    rclcpp::ParameterValue(params_.exit_behind_dist));
  declare_parameter_if_not_declared(
    node, plugin_name + ".stair_exit_ahead_dist", rclcpp::ParameterValue(params_.exit_ahead_dist));
  declare_parameter_if_not_declared(
    node, plugin_name + ".stair_policy_settle_time", rclcpp::ParameterValue(params_.settle_time));
  declare_parameter_if_not_declared(
    node, plugin_name + ".stair_policy_mode", rclcpp::ParameterValue(params_.stair_policy));
  declare_parameter_if_not_declared(
    node, plugin_name + ".flat_policy_mode", rclcpp::ParameterValue(params_.flat_policy));
  node->get_parameter(plugin_name + ".use_stair_mode", params_.enabled);
  node->get_parameter(plugin_name + ".stair_min_rise", params_.min_rise);
  node->get_parameter(plugin_name + ".stair_enter_dist", params_.enter_dist);
  node->get_parameter(plugin_name + ".stair_exit_behind_dist", params_.exit_behind_dist);
  node->get_parameter(plugin_name + ".stair_exit_ahead_dist", params_.exit_ahead_dist);
  node->get_parameter(plugin_name + ".stair_policy_settle_time", params_.settle_time);
  node->get_parameter(plugin_name + ".stair_policy_mode", params_.stair_policy);
  node->get_parameter(plugin_name + ".flat_policy_mode", params_.flat_policy);

  // Relative names: /<robot_id>/set_policy_mode and /<robot_id>/mode, both
  // syncai_driver_manager's. `mode` is published reliable by the Rust driver and
  // best effort by older ones; a best-effort reader takes either.
  policy_client_ = node->create_client<syncai_common::srv::SetPolicyMode>("set_policy_mode");
  mode_sub_ = node->create_subscription<std_msgs::msg::Int32MultiArray>(
    "mode", rclcpp::SensorDataQoS(), [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
      if (!msg->data.empty()) {
        reported_policy_.store(msg->data[0]);
      }
    });
  stair_mode_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "stair_mode", rclcpp::QoS(1).reliable().transient_local());

  send_policy_ = [this](int policy) {
    auto request = std::make_shared<syncai_common::srv::SetPolicyMode::Request>();
    request->mode = static_cast<uint8_t>(policy);
    policy_client_->async_send_request(request);
  };
  service_ready_ = [this]() { return policy_client_->service_is_ready(); };
  publish_ = [this](bool on_stairs) {
    std_msgs::msg::Bool msg;
    msg.data = on_stairs;
    stair_mode_pub_->publish(msg);
  };
  publish();
}

void StairMode::configure(
  const Params & params, SendPolicyFn send_policy, ServiceReadyFn service_ready,
  PublishFn publish_fn)
{
  params_ = params;
  send_policy_ = std::move(send_policy);
  service_ready_ = std::move(service_ready);
  publish_ = std::move(publish_fn);
  publish();
}

void StairMode::setPlan(const nav_msgs::msg::Path & full_plan)
{
  full_plan_ = full_plan;
  plan_has_height_ = pathHasHeight(full_plan);
}

StairMode::Decision StairMode::update(
  const rclcpp::Time & now, size_t remaining_poses, double stop_dist, double body_reach)
{
  (void)now;
  Decision d;
  if (!params_.enabled) {
    return d;
  }

  // A change of mode is made standing still. The controller brakes (its
  // acceleration clamps shape it), the policy goes out once the robot is
  // commanded to a stop (standing()), and it stands for settle_time before
  // moving off under the new one. A transition whose policy has not gone out
  // yet is dropped if the plan stops asking for it (a replan round the
  // stairs); once sent, it completes.
  // Off the stairs the window is looked for past the controller's stopping
  // distance. While an entry brakes, that figure falls with the speed, and
  // near the end of the brake faster than the distance still to go; with the
  // window sliding back off the stair the entry would be dropped (its policy
  // is not sent before the robot stands) and the robot drive on. So an entry
  // keeps the lead it started with. And the lead is never shorter than the
  // footprint's reach (2026-10-22; the header has the robot01 stall): a
  // controller that stops on its own at a stair's lethal cells takes its
  // stopping distance to 0 with it, and the window must still reach the
  // flight from the body's front edge.
  const double lead = std::max(
    {stop_dist, body_reach, transition_ == Transition::Entering ? entry_lead_ : 0.0, 0.0});
  const bool want = wantStairs(remaining_poses, lead);
  if (transition_ == Transition::None && want != on_stairs_) {
    int target = -1;
    if (want) {
      // Back to what flat_policy names, else to what the controller reported
      // before the switch; with neither known, leaving the stairs keeps the
      // stair policy rather than guessing an index.
      const int reported = reported_policy_.load();
      restore_policy_ = params_.flat_policy >= 0 ? params_.flat_policy : reported;
      // Already on it (the controller says so): no stop, nothing to send.
      target = reported == params_.stair_policy ? -1 : params_.stair_policy;
    } else {
      target = restore_policy_;
    }
    if (target < 0 || (!want && target == params_.stair_policy)) {
      // Nothing to switch: the controller changes speed and carrot only,
      // without stopping.
      on_stairs_ = want;
      publish();
      RCLCPP_INFO(logger_, "%s stair mode (no policy switch)", want ? "Entering" : "Leaving");
    } else {
      if (want && !(service_ready_ ? service_ready_() : true)) {
        // set_policy_mode is the only thing that puts the gait controller on
        // its stair policy; climbing on the level-ground one is not what this
        // mode promises. Fail the cycle; the server stops the robot.
        d.refuse =
          "Stairs ahead but set_policy_mode is unavailable; not climbing without the stair "
          "policy";
        return d;
      }
      transition_ = want ? Transition::Entering : Transition::Leaving;
      policy_sent_ = false;
      entry_lead_ = lead;
      RCLCPP_INFO(
        logger_, "%s stair mode: stopping to switch the policy to %d",
        want ? "Entering" : "Leaving", target);
    }
  }
  if (transition_ != Transition::None) {
    const bool entering = transition_ == Transition::Entering;
    if (!policy_sent_ && want != entering) {
      transition_ = Transition::None;
    } else {
      d.hold = true;
      d.leaving = !entering;
    }
  }
  d.on_stairs = on_stairs_;
  return d;
}

int StairMode::slopeAhead(
  const nav_msgs::msg::Path & path, size_t idx, double min_rise, double max_look)
{
  const auto & P = path.poses;
  if (idx >= P.size()) {
    return 0;
  }
  const double z0 = P[idx].pose.position.z;
  double d = 0.0;
  for (size_t k = idx; k + 1 < P.size() && d <= max_look; ++k) {
    d += euclidean_distance(P[k].pose, P[k + 1].pose);
    const double dz = P[k + 1].pose.position.z - z0;
    if (std::fabs(dz) >= min_rise) {
      return dz > 0.0 ? 1 : -1;
    }
  }
  return 0;
}

int StairMode::slopeAhead(size_t remaining_poses, double max_look) const
{
  if (!plan_has_height_ || full_plan_.poses.size() < remaining_poses) {
    return 0;
  }
  return slopeAhead(
    full_plan_, full_plan_.poses.size() - remaining_poses, params_.min_rise, max_look);
}

void StairMode::standing(const rclcpp::Time & now)
{
  if (transition_ == Transition::None) {
    return;
  }
  const bool entering = transition_ == Transition::Entering;
  if (!policy_sent_) {
    sendPolicy(entering ? params_.stair_policy : restore_policy_);
    policy_sent_ = true;
    settle_until_ = now + rclcpp::Duration::from_seconds(params_.settle_time);
  } else if (now >= settle_until_) {
    on_stairs_ = entering;
    transition_ = Transition::None;
    publish();
    RCLCPP_INFO(logger_, "%s stair mode: policy settled", entering ? "In" : "Out of");
  }
}

void StairMode::reset()
{
  if (transition_ != Transition::None && policy_sent_) {
    on_stairs_ = transition_ == Transition::Entering;
    publish();
  }
  transition_ = Transition::None;
  policy_sent_ = false;
}

double StairMode::stairRise(
  const nav_msgs::msg::Path & path, size_t idx, double behind, double ahead)
{
  const auto & poses = path.poses;
  if (idx >= poses.size()) {
    return 0.0;
  }
  double lo = poses[idx].pose.position.z;
  double hi = lo;
  double d = 0.0;
  for (size_t k = idx; k > 0 && d <= behind; --k) {
    d += euclidean_distance(poses[k].pose, poses[k - 1].pose);
    if (d <= behind) {
      lo = std::min(lo, poses[k - 1].pose.position.z);
      hi = std::max(hi, poses[k - 1].pose.position.z);
    }
  }
  d = 0.0;
  for (size_t k = idx; k + 1 < poses.size() && d <= ahead; ++k) {
    d += euclidean_distance(poses[k].pose, poses[k + 1].pose);
    if (d <= ahead) {
      lo = std::min(lo, poses[k + 1].pose.position.z);
      hi = std::max(hi, poses[k + 1].pose.position.z);
    }
  }
  return hi - lo;
}

double StairMode::stairRiseAhead(
  const nav_msgs::msg::Path & path, size_t idx, double lead, double ahead)
{
  double rise = stairRise(path, idx, 0.0, ahead);
  double d = 0.0;
  for (size_t k = idx; k + 1 < path.poses.size(); ++k) {
    d += euclidean_distance(path.poses[k].pose, path.poses[k + 1].pose);
    if (d > lead) {
      break;
    }
    rise = std::max(rise, stairRise(path, k + 1, 0.0, ahead));
  }
  return rise;
}

double StairMode::bodyReach(const std::vector<geometry_msgs::msg::Point> & footprint)
{
  double reach = 0.0;
  for (const auto & p : footprint) {
    reach = std::max(reach, std::fabs(p.x));
  }
  return reach;
}

bool StairMode::wantStairs(size_t remaining_poses, double lead) const
{
  // A level path (every 2D planner) never asks for stairs, which also takes a
  // robot left in stair mode by a cancelled goal back out on the next one.
  if (!plan_has_height_ || full_plan_.poses.size() < remaining_poses) {
    return false;
  }
  // The controller pruned everything before the pose nearest the robot.
  const size_t idx = full_plan_.poses.size() - remaining_poses;
  if (!on_stairs_) {
    return stairRiseAhead(full_plan_, idx, lead, params_.enter_dist) >= params_.min_rise;
  }
  return stairRise(full_plan_, idx, params_.exit_behind_dist, params_.exit_ahead_dist) >=
         params_.min_rise;
}

void StairMode::sendPolicy(int policy)
{
  if (policy < 0 || policy > 255) {
    return;
  }
  if (service_ready_ && !service_ready_()) {
    // Only reachable on the way off the stairs (entering checks first): the
    // robot walks on under the stair policy, which is the safer of the two to
    // be left in.
    RCLCPP_WARN(
      logger_, "set_policy_mode unavailable; policy %d not sent, staying on the current one",
      policy);
    return;
  }
  if (send_policy_) {
    send_policy_(policy);
  }
  RCLCPP_INFO(logger_, "set_policy_mode %d sent", policy);
}

void StairMode::publish()
{
  if (publish_) {
    publish_(on_stairs_);
  }
}

bool StairMode::findCorner(
  const nav_msgs::msg::Path & path, double min_turn, double window, size_t & index,
  double & heading_after)
{
  const auto & P = path.poses;
  const size_t n = P.size();
  if (n < 3) {
    return false;
  }
  // The heading from pose i over `window` metres of path, forward (dir > 0)
  // or back; false when there is less than half a window of path that way.
  const auto heading = [&](size_t i, int dir, double & h) {
    double d = 0.0;
    size_t k = i;
    while ((dir > 0 && k + 1 < n) || (dir < 0 && k > 0)) {
      const size_t nk = dir > 0 ? k + 1 : k - 1;
      d += euclidean_distance(P[k].pose, P[nk].pose);
      k = nk;
      if (d >= window) {
        break;
      }
    }
    if (k == i || d < window / 2) {
      return false;
    }
    const auto & a = dir > 0 ? P[i].pose.position : P[k].pose.position;
    const auto & b = dir > 0 ? P[k].pose.position : P[i].pose.position;
    h = std::atan2(b.y - a.y, b.x - a.x);
    return true;
  };
  bool in_bend = false;
  double sharpest = 0.0;
  for (size_t i = 1; i + 1 < n; ++i) {
    double hb, ha;
    if (!heading(i, -1, hb) || !heading(i, 1, ha)) {
      continue;
    }
    const double turn = std::fabs(std::remainder(ha - hb, 2.0 * M_PI));
    if (turn > min_turn) {
      if (!in_bend || turn > sharpest) {
        sharpest = turn;
        index = i;
        heading_after = ha;
      }
      in_bend = true;
    } else if (in_bend) {
      break;  // the first bend only
    }
  }
  return in_bend;
}

}  // namespace syncai_util
