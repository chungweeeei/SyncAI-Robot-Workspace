#ifndef SYNCAI_NAV2_COSTMAP_2D__COSTMAP_2D_PUBLISHER_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__COSTMAP_2D_PUBLISHER_HPP_

#include <algorithm>
#include <memory>
#include <string>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "syncai_costmap_2d/costmap_2d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace syncai_costmap_2d
{

class Costmap2DPublisher
{
public:
  Costmap2DPublisher(
    const rclcpp::Node::SharedPtr & node, Costmap2D * costmap, std::string global_frame,
    std::string topic_name, bool always_send_full_costmap = false);

  ~Costmap2DPublisher();

  void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
  {
    x0_ = std::min(x0, x0_);
    xn_ = std::max(xn, xn_);
    y0_ = std::min(y0, y0_);
    yn_ = std::max(yn, yn_);
  }

  void publishCostmap();

private:
  void prepareGrid();
  void prepareCostmap();

  void costmap_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> request,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> response);

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("syncai_costmap_2d")};

  Costmap2D * costmap_;
  std::string global_frame_;
  std::string topic_name_;
  unsigned int x0_, y0_, xn_, yn_;
  double saved_origin_x_, saved_origin_y_;
  bool always_send_full_costmap_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_update_pub_;

  // Publisher for raw costmap values
  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_raw_pub_;

  // Service for getting the costmaps
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_service_;

  float grid_resolution;
  unsigned int grid_width, grid_height;

  std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid_;
  std::unique_ptr<nav2_msgs::msg::Costmap> costmap_raw_;
  // Translate from 0-255 values in costmap to -1 to 100 values in message.
  static char * cost_translation_table_;
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D__COSTMAP_2D_PUBLISHER_HPP_