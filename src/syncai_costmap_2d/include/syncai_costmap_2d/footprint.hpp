#ifndef SYNCIAI_NAV2_COSTMAP_2D__FOOTPRINT_HPP_
#define SYNCIAI_NAV2_COSTMAP_2D__FOOTPRINT_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace syncai_costmap_2d
{
/**
 * @brief Calculate the extreme distances from the base of the robot to the footprint.
 */
void calculateMinAndMaxDistances(
  const std::vector<geometry_msgs::msg::Point> & footprint, double & min_dist, double & max_dist);

/**
 * @brief Convert Point32 to Point
 */
geometry_msgs::msg::Point toPoint(geometry_msgs::msg::Point32 pt);

/**
 * @brief Convert Point to Point32
 */
geometry_msgs::msg::Point32 toPoint32(geometry_msgs::msg::Point pt);

/**
 * @brief Convert vector of Points to Polygon msg
 */
geometry_msgs::msg::Polygon toPolygon(std::vector<geometry_msgs::msg::Point> pts);

/**
 * @brief Convert Polygon msg to vector of Points.
 */
std::vector<geometry_msgs::msg::Point> toPointVector(
  geometry_msgs::msg::Polygon::SharedPtr polygon);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (list of Points)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  std::vector<geometry_msgs::msg::Point> & oriented_footprint);

/**
 * @brief  Given a pose and base footprint, build the oriented footprint of the robot (PolygonStamped)
 * @param  x The x position of the robot
 * @param  y The y position of the robot
 * @param  theta The orientation of the robot
 * @param  footprint_spec Basic shape of the footprint
 * @param  oriented_footprint Will be filled with the points in the oriented footprint of the robot
*/
void transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  geometry_msgs::msg::PolygonStamped & oriented_footprint);

/**
 * @brief Adds the specified amount of padding to the footprint (in place)
 */
void padFootprint(std::vector<geometry_msgs::msg::Point> & footprint, double padding);

/**
 * @brief Create a circular footprint from a given radius
 */
std::vector<geometry_msgs::msg::Point> makeFootprintFromRadius(double radius);

/**
 * @brief Make the footprint from the given string.
 *
 * Format should be bracketed array of arrays of floats, like so: [[1.0, 2.2], [3.3, 4.2], ...]
 *
 */
bool makeFootprintFromString(
  const std::string & footprint_string, std::vector<geometry_msgs::msg::Point> & footprint);

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D__FOOTPRINT_HPP_