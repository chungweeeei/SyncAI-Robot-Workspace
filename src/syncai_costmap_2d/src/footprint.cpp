#include "syncai_costmap_2d/footprint.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point32.hpp"
#include "syncai_costmap_2d/array_parser.hpp"
#include "syncai_costmap_2d/costmap_math.hpp"

namespace syncai_costmap_2d
{
void calculateMinAndMaxDistances(
  const std::vector<geometry_msgs::msg::Point> & footprint, double & min_dist, double & max_dist)
{
  min_dist = std::numeric_limits<double>::max();
  max_dist = 0.0;

  if (footprint.size() <= 2) {
    return;
  }

  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
    // check the distance from the robot center point to the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
    double edge_dist = distanceToLine(
      0.0, 0.0, footprint[i].x, footprint[i].y, footprint[i + 1].x, footprint[i + 1].y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
  }

  // we also need to do the last vertex and the first vertex
  double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
  double edge_dist = distanceToLine(
    0.0, 0.0, footprint.back().x, footprint.back().y, footprint.front().x, footprint.front().y);
  min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
  max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

geometry_msgs::msg::Point32 toPoint32(geometry_msgs::msg::Point pt)
{
  geometry_msgs::msg::Point32 point32;
  point32.x = pt.x;
  point32.y = pt.y;
  point32.z = pt.z;
  return point32;
}

geometry_msgs::msg::Point toPoint(geometry_msgs::msg::Point32 pt)
{
  geometry_msgs::msg::Point point;
  point.x = pt.x;
  point.y = pt.y;
  point.z = pt.z;
  return point;
}

geometry_msgs::msg::Polygon toPolygon(std::vector<geometry_msgs::msg::Point> pts)
{
  geometry_msgs::msg::Polygon polygon;
  for (unsigned int i = 0; i < pts.size(); i++) {
    polygon.points.push_back(toPoint32(pts[i]));
  }
  return polygon;
}

std::vector<geometry_msgs::msg::Point> toPointVector(geometry_msgs::msg::Polygon::SharedPtr polygon)
{
  std::vector<geometry_msgs::msg::Point> pts;
  for (unsigned int i = 0; i < polygon->points.size(); i++) {
    pts.push_back(toPoint(polygon->points[i]));
  }
  return pts;
}

void transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.resize(footprint_spec.size());
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    double new_x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    double new_y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    geometry_msgs::msg::Point & new_pt = oriented_footprint[i];
    new_pt.x = new_x;
    new_pt.y = new_y;
  }
}

void transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  geometry_msgs::msg::PolygonStamped & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.polygon.points.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::msg::Point32 new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.polygon.points.push_back(new_pt);
  }
}

void padFootprint(std::vector<geometry_msgs::msg::Point> & footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++) {
    geometry_msgs::msg::Point & pt = footprint[i];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

std::vector<geometry_msgs::msg::Point> makeFootprintFromRadius(double radius)
{
  std::vector<geometry_msgs::msg::Point> points;

  // Loop over 16 angles around a circle making a point each time
  int N = 16;
  geometry_msgs::msg::Point pt;
  for (int i = 0; i < N; ++i) {
    double angle = i * 2 * M_PI / N;
    pt.x = cos(angle) * radius;
    pt.y = sin(angle) * radius;

    points.push_back(pt);
  }

  return points;
}

bool makeFootprintFromString(
  const std::string & footprint_string, std::vector<geometry_msgs::msg::Point> & footprint)
{
  std::string error;
  std::vector<std::vector<float>> vvf = parseVVF(footprint_string, error);

  if (error != "") {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_costmap_2d"), "Error parsing footprint parameter: '%s'",
      error.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_costmap_2d"), "  Footprint string was '%s'.",
      footprint_string.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3) {
    RCLCPP_ERROR(
      rclcpp::get_logger("nav2_costmap_2d"),
      "You must specify at least three points for the robot footprint, reverting to previous "
      "footprint.");  //NOLINT
    return false;
  }

  footprint.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      geometry_msgs::msg::Point point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      footprint.push_back(point);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("nav2_costmap_2d"),
        "Points in the footprint specification must be pairs of numbers. Found a point with %d "
        "numbers.",  //NOLINT
        static_cast<int>(vvf[i].size()));
      return false;
    }
  }

  return true;
}
}  // namespace syncai_costmap_2d