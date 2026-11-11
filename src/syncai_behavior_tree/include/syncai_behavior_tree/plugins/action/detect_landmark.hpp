#ifndef SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_LANDMARK_HPP_
#define SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_LANDMARK_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_common/srv/detect_landmark.hpp"

namespace syncai_behavior_tree
{

/**
 * @brief Poll the landmark detector (detect_landmark, syncai_common/srv/
 * DetectLandmark) and act on the sightings the tree TRUSTS. One class, two
 * tags (charge.xml, 2026-11):
 *
 *  - DetectLandmark: RUNNING until the first trusted sighting, then SUCCESS
 *    with it on `landmark_pose`; FAILURE after `timeout`. The check that the
 *    landmark is in view from the staging pose, before the gait is switched
 *    for an approach that would only end LOST.
 *  - PublishLandmark: RUNNING for as long as it is ticked, publishing every
 *    trusted sighting on `topic` (geometry_msgs/PoseStamped, the sensor's
 *    frame, stamped with its image) -- the pose stream TrackPose follows. It
 *    never succeeds: it runs under a Parallel beside TrackPose, whose success
 *    halts it. FAILURE only when detection cannot work at all: the landmark
 *    is not in the detector's registry, or the detector has not given one
 *    usable answer for `sensor_timeout` (no service, or SENSOR_UNAVAILABLE).
 *    A landmark that is merely out of view is not its failure -- nothing is
 *    published, and track_pose's pose_timeout / lost_timeout judge that.
 *
 * The judging is here because the detector only detects (DetectLandmark.srv)
 * and the controller only drives: a sighting is trusted when it is not
 * `ambiguous` (ArUco holds a mirrored solution it has not ruled out),
 * follows `min_consecutive` sightings in a row, and fits within
 * `max_reprojection_error_px` when that is set. One sighting is published
 * once: the detector answers at once with its newest, and this polls faster
 * than the camera delivers.
 *
 * The pose is asked for in the SENSOR's frame (target_frame empty): the
 * detector then needs no TF and answers in every state, and track_pose takes
 * the pose into odom at its stamp itself, waiting for an odometry that lags
 * the image where the detector would have answered TRANSFORM_FAILED.
 *
 * One request in flight at a time, on its own callback group and executor
 * spun without blocking, as WaitForMotionState: a tick never waits for the
 * detector. The service is not waited for at construction (BtServiceNode
 * does, which would fail every tree load on a robot without a detector).
 *
 * When it ends it logs what the detector said over the run -- how many
 * sightings, how many of them ambiguous or too short a streak -- which is
 * the number the trust thresholds are tuned by.
 */
class DetectLandmark : public BT::StatefulActionNode
{
public:
  /**
   * @param stream false: DetectLandmark; true: PublishLandmark
   */
  DetectLandmark(const std::string & name, const BT::NodeConfiguration & conf, bool stream);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "landmark_id", "charger_a", "Name in the detector's landmark registry"),
      BT::InputPort<std::string>("service_name", "detect_landmark", "The detector's service"),
      BT::InputPort<double>(
        "max_age", 0.0, "Oldest sighting accepted [s]; <= 0 = the detector's default"),
      BT::InputPort<int>(
        "min_consecutive", 3, "Sightings in a row a trusted sighting needs behind it"),
      BT::InputPort<double>(
        "max_reprojection_error_px", 0.0,
        "Largest fit residual trusted [px]; <= 0 = not judged"),
      BT::InputPort<bool>(
        "accept_ambiguous", false,
        "Trust a sighting the detector still holds an alternative for"),
      BT::InputPort<int>(
        "timeout", 5000, "DetectLandmark: milliseconds to the first trusted sighting"),
      BT::InputPort<int>(
        "sensor_timeout", 5000,
        "PublishLandmark: milliseconds without one usable answer from the detector"),
      BT::InputPort<std::string>(
        "topic", "tracked_landmark", "PublishLandmark: the PoseStamped topic, relative"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "landmark_pose", "DetectLandmark: the trusted sighting, in the sensor's frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using Service = syncai_common::srv::DetectLandmark;

  // Take a finished answer if there is one, then send the next request.
  // Returns true when `sighting` holds a new trusted sighting.
  bool poll(geometry_msgs::msg::PoseStamped & sighting);
  bool trusted(const Service::Response & response);
  void logSummary(const char * why);
  BT::NodeStatus fail(const std::string & msg);

  const bool stream_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Client<Service>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string service_name_;
  std::string topic_;

  // Per run (onStart()).
  std::string landmark_id_;
  double max_age_{0.0};
  int min_consecutive_{3};
  double max_reprojection_error_px_{0.0};
  bool accept_ambiguous_{false};
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point deadline_;
  std::chrono::milliseconds sensor_timeout_{5000};
  // The last time the detector gave an answer it could have seen the
  // landmark in (anything but no answer / SENSOR_UNAVAILABLE).
  std::chrono::steady_clock::time_point last_usable_;

  bool request_pending_{false};
  std::chrono::steady_clock::time_point request_sent_;
  rclcpp::Client<Service>::SharedFuture future_;
  // The image stamp of the last sighting acted on [ns]: one sighting, once.
  int64_t last_stamp_ns_{0};
  // Set by an answer that ends the run (UNKNOWN_LANDMARK).
  std::string fatal_;
  // The detector's last word that was not a trusted sighting, for the
  // failure message and the throttled log.
  std::string last_reason_;

  // What the detector said over the run.
  unsigned answers_{0};
  unsigned sightings_{0};
  unsigned ambiguous_{0};
  unsigned short_streak_{0};
  unsigned poor_fit_{0};
  unsigned used_{0};
};

}  // namespace syncai_behavior_tree

#endif  // SYNCAI_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_LANDMARK_HPP_
