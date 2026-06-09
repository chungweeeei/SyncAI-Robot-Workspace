#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "syncai_costmap_2d/cost_values.hpp"
#include "syncai_costmap_2d/costmap_topic_collision_checker.hpp"
#include "syncai_costmap_2d/exceptions.hpp"
#include "syncai_costmap_2d/footprint.hpp"
#include "syncai_util/line_iterator.hpp"

using namespace std::chrono_literals;

namespace syncai_costmap_2d
{

CostmapTopicCollisionChecker::CostmapTopicCollisionChecker(
  CostmapSubscriber & costmap_sub, FootprintSubscriber & footprint_sub, std::string name)
: name_(name), costmap_sub_(costmap_sub), footprint_sub_(footprint_sub), collision_checker_(nullptr)
{
}

bool CostmapTopicCollisionChecker::isCollisionFree(
  const geometry_msgs::msg::Pose2D & pose, bool fetch_costmap_and_footprint)
{
  try {
    if (scorePose(pose, fetch_costmap_and_footprint) >= LETHAL_OBSTACLE) {
      return false;
    }
    return true;
  } catch (const IllegalPoseException & e) {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } catch (const CollisionCheckerException & e) {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "%s", e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger(name_), "Failed to check pose score!");
    return false;
  }
}

double CostmapTopicCollisionChecker::scorePose(
  const geometry_msgs::msg::Pose2D & pose, bool fetch_costmap_and_footprint)
{
  if (fetch_costmap_and_footprint) {
    try {
      collision_checker_.setCostmap(costmap_sub_.getCostmap());
    } catch (const std::runtime_error & e) {
      throw CollisionCheckerException(e.what());
    }
  }

  unsigned int cell_x, cell_y;
  if (!collision_checker_.worldToMap(pose.x, pose.y, cell_x, cell_y)) {
    RCLCPP_DEBUG(rclcpp::get_logger(name_), "Map Cell: [%d, %d]", cell_x, cell_y);
    throw IllegalPoseException(name_, "Pose Goes Off Grid.");
  }

  return collision_checker_.footprintCost(getFootprint(pose, fetch_costmap_and_footprint));
}

Footprint CostmapTopicCollisionChecker::getFootprint(
  const geometry_msgs::msg::Pose2D & pose, bool fetch_latest_footprint)
{
  if (fetch_latest_footprint) {
    std_msgs::msg::Header header;
    if (!footprint_sub_.getFootprintInRobotFrame(footprint_, header)) {
      throw CollisionCheckerException("Current footprint not available.");
    }
  }
  Footprint footprint;
  transformFootprint(pose.x, pose.y, pose.theta, footprint_, footprint);

  return footprint;
}

}  // namespace syncai_costmap_2d
