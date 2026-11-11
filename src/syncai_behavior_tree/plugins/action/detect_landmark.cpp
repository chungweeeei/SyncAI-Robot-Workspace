#include "syncai_behavior_tree/plugins/action/detect_landmark.hpp"

#include <chrono>
#include <cstdio>
#include <future>
#include <memory>
#include <string>

#include "syncai_behavior_tree/blackboard_keys.hpp"

namespace syncai_behavior_tree
{

namespace
{

// An answer that has not come back in this long is given up on, so a
// detector that died mid-call does not hold the one request slot for good.
constexpr std::chrono::milliseconds kAnswerTimeout{1000};

double secondsSince(std::chrono::steady_clock::time_point then)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - then).count();
}

}  // namespace

DetectLandmark::DetectLandmark(
  const std::string & name, const BT::NodeConfiguration & conf, bool stream)
: BT::StatefulActionNode(name, conf), stream_(stream)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  service_name_ = "detect_landmark";
  getInput("service_name", service_name_);
  client_ = node_->create_client<Service>(
    service_name_, rclcpp::ServicesQoS().get_rmw_qos_profile(), callback_group_);

  if (stream_) {
    topic_ = "tracked_landmark";
    getInput("topic", topic_);
    // Reliable, depth 10: track_pose subscribes best effort, which matches.
    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_, rclcpp::QoS(10));
  }
}

BT::NodeStatus DetectLandmark::onStart()
{
  landmark_id_ = "charger_a";
  getInput("landmark_id", landmark_id_);
  getInput("max_age", max_age_);
  getInput("min_consecutive", min_consecutive_);
  getInput("max_reprojection_error_px", max_reprojection_error_px_);
  getInput("accept_ambiguous", accept_ambiguous_);
  int timeout_ms = 5000;
  int sensor_timeout_ms = 5000;
  getInput("timeout", timeout_ms);
  getInput("sensor_timeout", sensor_timeout_ms);
  if (landmark_id_.empty() || timeout_ms <= 0 || sensor_timeout_ms <= 0) {
    return fail("landmark_id must be set, and timeout and sensor_timeout > 0");
  }

  start_ = std::chrono::steady_clock::now();
  deadline_ = start_ + std::chrono::milliseconds(timeout_ms);
  sensor_timeout_ = std::chrono::milliseconds(sensor_timeout_ms);
  last_usable_ = start_;
  // A request left in flight by a halted run answers for another moment's
  // question; its answer is dropped.
  request_pending_ = false;
  fatal_.clear();
  last_reason_ = "no answer from " + service_name_ + " yet";
  answers_ = sightings_ = ambiguous_ = short_streak_ = poor_fit_ = used_ = 0;

  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    stream_ ? "Following the charger's marker" : "Looking for the charger's marker");
  return onRunning();
}

BT::NodeStatus DetectLandmark::onRunning()
{
  geometry_msgs::msg::PoseStamped sighting;
  const bool have = poll(sighting);

  if (!fatal_.empty()) {
    logSummary("failed");
    return fail(fatal_);
  }

  if (stream_) {
    if (have) {
      publisher_->publish(sighting);
    }
    if (std::chrono::steady_clock::now() - last_usable_ > sensor_timeout_) {
      logSummary("failed");
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.1f", secondsSince(last_usable_));
      return fail(
        "the landmark detector has given no usable answer for " + std::string(buf) + " s: " +
        last_reason_);
    }
    return BT::NodeStatus::RUNNING;
  }

  if (have) {
    setOutput("landmark_pose", sighting);
    RCLCPP_INFO(
      node_->get_logger(),
      "[DetectLandmark] '%s' in view after %.2f s: (%.3f, %.3f, %.3f) in '%s'",
      landmark_id_.c_str(), secondsSince(start_), sighting.pose.position.x,
      sighting.pose.position.y, sighting.pose.position.z, sighting.header.frame_id.c_str());
    logSummary("found");
    return BT::NodeStatus::SUCCESS;
  }
  if (std::chrono::steady_clock::now() >= deadline_) {
    logSummary("failed");
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.1f", secondsSince(start_));
    return fail(
      "no trusted sighting of landmark '" + landmark_id_ + "' in " + std::string(buf) +
      " s; the detector's last word: " + last_reason_);
  }
  return BT::NodeStatus::RUNNING;
}

void DetectLandmark::onHalted()
{
  // PublishLandmark's ordinary end: TrackPose finished and the Parallel
  // halted this node.
  logSummary("halted");
  request_pending_ = false;
}

bool DetectLandmark::poll(geometry_msgs::msg::PoseStamped & sighting)
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));
  bool have = false;

  if (request_pending_) {
    if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      request_pending_ = false;
      const auto response = future_.get();
      ++answers_;
      if (response->error_code == Service::Response::NONE) {
        last_usable_ = std::chrono::steady_clock::now();
        const int64_t stamp_ns = rclcpp::Time(response->pose.header.stamp).nanoseconds();
        if (stamp_ns != last_stamp_ns_) {
          // A sighting not judged before.
          last_stamp_ns_ = stamp_ns;
          ++sightings_;
          if (trusted(*response)) {
            ++used_;
            sighting = response->pose;
            have = true;
          }
        }
      } else if (response->error_code == Service::Response::UNKNOWN_LANDMARK) {
        fatal_ = "landmark '" + landmark_id_ + "' is not in the detector's registry: " +
                 response->error_msg;
      } else {
        // SENSOR_STARTING, NOT_DETECTED and TRANSFORM_FAILED are a detector
        // that works and has nothing to show; SENSOR_UNAVAILABLE is one that
        // cannot see at all, and is not "usable".
        if (response->error_code != Service::Response::SENSOR_UNAVAILABLE) {
          last_usable_ = std::chrono::steady_clock::now();
        }
        last_reason_ = "error_code " + std::to_string(response->error_code) + ": " +
                       response->error_msg;
      }
    } else if (std::chrono::steady_clock::now() - request_sent_ > kAnswerTimeout) {
      request_pending_ = false;
      // The client keeps a request until its answer comes; this one's will not.
      client_->prune_pending_requests();
      last_reason_ = service_name_ + " did not answer a request within " +
                     std::to_string(kAnswerTimeout.count()) + " ms";
    }
  }

  if (!request_pending_) {
    if (client_->service_is_ready()) {
      auto request = std::make_shared<Service::Request>();
      request->landmark_id = landmark_id_;
      // Empty: the sensor's own frame (see the class comment).
      request->target_frame = "";
      request->max_age = static_cast<float>(max_age_);
      future_ = client_->async_send_request(request).share();
      request_sent_ = std::chrono::steady_clock::now();
      request_pending_ = true;
    } else {
      last_reason_ = "service " + std::string(client_->get_service_name()) +
                     " is not available (is landmark_detector running?)";
    }
  }

  if (!have) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] no trusted sighting of '%s' yet: %s (%u answers, %u sightings: %u ambiguous, %u "
      "under %d in a row, %u poor fit)",
      registrationName().c_str(), landmark_id_.c_str(), last_reason_.c_str(), answers_,
      sightings_, ambiguous_, short_streak_, poor_fit_, min_consecutive_);
  }
  return have;
}

bool DetectLandmark::trusted(const Service::Response & response)
{
  char buf[160];
  if (response.ambiguous && !accept_ambiguous_) {
    ++ambiguous_;
    last_reason_ = "the newest sighting is ambiguous (the detector holds a second pose it has "
                   "not ruled out)";
    return false;
  }
  if (static_cast<int>(response.consecutive) < min_consecutive_) {
    ++short_streak_;
    std::snprintf(
      buf, sizeof(buf), "the newest sighting follows %u in a row, under min_consecutive %d",
      static_cast<unsigned>(response.consecutive), min_consecutive_);
    last_reason_ = buf;
    return false;
  }
  if (
    max_reprojection_error_px_ > 0.0 && response.reprojection_error_px >= 0.0f &&
    static_cast<double>(response.reprojection_error_px) > max_reprojection_error_px_) {
    ++poor_fit_;
    std::snprintf(
      buf, sizeof(buf), "the newest sighting's fit is %.2f px, over max_reprojection_error_px %.2f",
      static_cast<double>(response.reprojection_error_px), max_reprojection_error_px_);
    last_reason_ = buf;
    return false;
  }
  return true;
}

void DetectLandmark::logSummary(const char * why)
{
  // The numbers the trust thresholds are tuned by: how much of what the
  // detector saw was thrown away, and for which reason.
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] %s after %.1f s on '%s': %u answers, %u sightings, %u trusted; dropped %u ambiguous "
    "(%.0f %%), %u under %d in a row, %u poor fit",
    registrationName().c_str(), why, secondsSince(start_), landmark_id_.c_str(), answers_,
    sightings_, used_, ambiguous_, sightings_ > 0 ? 100.0 * ambiguous_ / sightings_ : 0.0,
    short_streak_, min_consecutive_, poor_fit_);
}

BT::NodeStatus DetectLandmark::fail(const std::string & msg)
{
  RCLCPP_ERROR(node_->get_logger(), "[%s] %s", registrationName().c_str(), msg.c_str());
  // report_failure()'s contract (BtActionNode has it, a StatefulActionNode
  // does not): registration name + message, last writer wins.
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kFailedNode, registrationName());
  config().blackboard->set<std::string>(blackboard_keys::kFailureMsg, msg);  // NOLINT
  config().blackboard->set<std::string>(  // NOLINT
    blackboard_keys::kNavStatusMsg,
    stream_ ? "Lost the charger's marker detection" : "The charger's marker was not found");
  return BT::NodeStatus::FAILURE;
}

}  // namespace syncai_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder once = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::DetectLandmark>(name, config, false);
  };
  factory.registerBuilder<syncai_behavior_tree::DetectLandmark>("DetectLandmark", once);

  BT::NodeBuilder stream = [](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<syncai_behavior_tree::DetectLandmark>(name, config, true);
  };
  factory.registerBuilder<syncai_behavior_tree::DetectLandmark>("PublishLandmark", stream);
}
