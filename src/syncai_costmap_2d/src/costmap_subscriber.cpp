#include "syncai_costmap_2d/costmap_subscriber.hpp"

#include <memory>
#include <string>

namespace syncai_costmap_2d
{
CostmapSubscriber::CostmapSubscriber(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name)
: topic_name_(topic_name)
{
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    topic_name_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CostmapSubscriber::costmapCallback, this, std::placeholders::_1));
}

std::shared_ptr<Costmap2D> CostmapSubscriber::getCostmap()
{
  if (!costmap_received_) {
    throw std::runtime_error("Costmap is not available");
  }

  toCostmap2D();
  return costmap_;
}

void CostmapSubscriber::toCostmap2D()
{
  auto current_costmap_msg = std::atomic_load(&costmap_msg_);

  if (costmap_ == nullptr) {
    costmap_ = std::make_shared<Costmap2D>(
      current_costmap_msg->metadata.size_x, current_costmap_msg->metadata.size_y,
      current_costmap_msg->metadata.resolution, current_costmap_msg->metadata.origin.position.x,
      current_costmap_msg->metadata.origin.position.y);
  } else if (
    costmap_->getSizeInCellsX() != current_costmap_msg->metadata.size_x ||  // NOLINT
    costmap_->getSizeInCellsY() != current_costmap_msg->metadata.size_y ||
    costmap_->getResolution() != current_costmap_msg->metadata.resolution ||
    costmap_->getOriginX() != current_costmap_msg->metadata.origin.position.x ||
    costmap_->getOriginY() != current_costmap_msg->metadata.origin.position.y) {
    // Update the size of the costmap
    costmap_->resizeMap(
      current_costmap_msg->metadata.size_x, current_costmap_msg->metadata.size_y,
      current_costmap_msg->metadata.resolution, current_costmap_msg->metadata.origin.position.x,
      current_costmap_msg->metadata.origin.position.y);
  }

  unsigned char * master_array = costmap_->getCharMap();
  unsigned int index = 0;
  for (unsigned int i = 0; i < current_costmap_msg->metadata.size_x; ++i) {
    for (unsigned int j = 0; j < current_costmap_msg->metadata.size_y; ++j) {
      master_array[index] = current_costmap_msg->data[index];
      ++index;
    }
  }
}

void CostmapSubscriber::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::atomic_store(&costmap_msg_, msg);
  if (!costmap_received_) {
    costmap_received_ = true;
  }
}
}  // namespace syncai_costmap_2d