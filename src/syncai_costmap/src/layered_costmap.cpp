#include "syncai_costmap/layered_costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "syncai_costmap/footprint.hpp"

using std::vector;

namespace syncai_costmap
{
LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
: combined_costmap_(),
  global_frame_(global_frame),
  rolling_window_(rolling_window),
  current_(false),
  minx_(0.0),
  miny_(0.0),
  maxx_(0.0),
  maxy_(0.0),
  bx0_(0),
  bxn_(0),
  by0_(0),
  byn_(0),
  initialized_(false),
  size_locked_(false),
  circumscribed_radius_(1.0),
  inscribed_radius_(0.1)
{
  if (track_unknown) {
    combined_costmap_.setDefaultValue(255);
  } else {
    combined_costmap_.setDefaultValue(0);
  }
}

LayeredCostmap::~LayeredCostmap()
{
  while (plugins_.size() > 0) {
    plugins_.pop_back();
  }
}

void LayeredCostmap::addPlugin(std::shared_ptr<Layer> plugin)
{
  std::unique_lock<Costmap::mutex_t> lock(*(combined_costmap_.getMutex()));
  plugins_.push_back(plugin);
}

void LayeredCostmap::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
  bool size_locked)
{
  std::unique_lock<Costmap::mutex_t> lock(*(combined_costmap_.getMutex()));
  size_locked_ = size_locked;
  combined_costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin) {
    (*plugin)->matchSize();
  }
}

bool LayeredCostmap::isOutofBounds(double robot_x, double robot_y)
{
  unsigned int mx, my;
  return !combined_costmap_.worldToMap(robot_x, robot_y, mx, my);
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
  // Lock for the remainder of this function, some plugins (e.g. VoxelLayer)
  // implement thread unsafe updateBounds() functions.
  std::unique_lock<Costmap::mutex_t> lock(*(combined_costmap_.getMutex()));

  // if we're using a rolling buffer costmap...
  // we need to update the origin using the robot's position
  if (rolling_window_) {
    double new_origin_x = robot_x - combined_costmap_.getSizeInMetersX() / 2;
    double new_origin_y = robot_y - combined_costmap_.getSizeInMetersY() / 2;
    combined_costmap_.updateOrigin(new_origin_x, new_origin_y);
  }

  // Once per state change, not per update (see robot_out_of_bounds_).
  const bool out_of_bounds = isOutofBounds(robot_x, robot_y);
  if (out_of_bounds != robot_out_of_bounds_) {
    robot_out_of_bounds_ = out_of_bounds;
    const double ox = combined_costmap_.getOriginX();
    const double oy = combined_costmap_.getOriginY();
    if (out_of_bounds) {
      RCLCPP_WARN(
        rclcpp::get_logger("syncai_costmap"),
        "Robot at (%.2f, %.2f) is out of bounds of the costmap (%.2f, %.2f) to (%.2f, %.2f) in "
        "%s; said once until it is back. Expected for the planner's global costmap, which has "
        "no layers and stays at its default size (Octo3D reads no cells)",
        robot_x, robot_y, ox, oy, ox + combined_costmap_.getSizeInMetersX(),
        oy + combined_costmap_.getSizeInMetersY(), global_frame_.c_str());
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("syncai_costmap"),
        "Robot at (%.2f, %.2f) is back inside the costmap in %s", robot_x, robot_y,
        global_frame_.c_str());
    }
  }

  if (plugins_.size() == 0) {
    return;
  }

  minx_ = miny_ = std::numeric_limits<double>::max();
  maxx_ = maxy_ = std::numeric_limits<double>::lowest();

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin) {
    double prev_minx = minx_;
    double prev_miny = miny_;
    double prev_maxx = maxx_;
    double prev_maxy = maxy_;
    (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx_, &miny_, &maxx_, &maxy_);
    if (minx_ > prev_minx || miny_ > prev_miny || maxx_ < prev_maxx || maxy_ < prev_maxy) {
      RCLCPP_WARN(
        rclcpp::get_logger("syncai_costmap"),
        "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
        "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
        prev_minx, prev_miny, prev_maxx, prev_maxy, minx_, miny_, maxx_, maxy_,
        (*plugin)->getName().c_str());
    }
  }
  int x0, xn, y0, yn;
  combined_costmap_.worldToMapEnforceBounds(minx_, miny_, x0, y0);
  combined_costmap_.worldToMapEnforceBounds(maxx_, maxy_, xn, yn);

  x0 = std::max(0, x0);
  xn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsX()), xn + 1);
  y0 = std::max(0, y0);
  yn = std::min(static_cast<int>(combined_costmap_.getSizeInCellsY()), yn + 1);

  RCLCPP_DEBUG(
    rclcpp::get_logger("syncai_costmap"), "Updating area x: [%d, %d] y: [%d, %d]", x0, xn, y0, yn);

  if (xn < x0 || yn < y0) {
    return;
  }

  // The window is reset and rewritten by each layer in order (upstream's
  // no-filter branch; its filter branch, a primary grid copied into the
  // master before the filters ran, went with the keepout filter).
  combined_costmap_.resetMap(x0, y0, xn, yn);
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin) {
    (*plugin)->updateCosts(combined_costmap_, x0, y0, xn, yn);
  }

  bx0_ = x0;
  bxn_ = xn;
  by0_ = y0;
  byn_ = yn;

  initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
  current_ = true;
  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin) {
    current_ = current_ && ((*plugin)->isCurrent() || !(*plugin)->isEnabled());
  }
  return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec)
{
  footprint_ = footprint_spec;
  syncai_costmap::calculateMinAndMaxDistances(
    footprint_spec, inscribed_radius_, circumscribed_radius_);

  for (vector<std::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end();
       ++plugin) {
    (*plugin)->onFootprintChanged();
  }
}

}  // namespace syncai_costmap
