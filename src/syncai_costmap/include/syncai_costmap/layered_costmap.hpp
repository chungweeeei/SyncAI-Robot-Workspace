#ifndef SYNCAI_COSTMAP__LAYERED_COSTMAP_HPP_
#define SYNCAI_COSTMAP__LAYERED_COSTMAP_HPP_

#include <memory>
#include <string>
#include <vector>

#include "syncai_costmap/cost_values.hpp"
#include "syncai_costmap/costmap_2d.hpp"
#include "syncai_costmap/layer.hpp"

namespace syncai_costmap
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
class LayeredCostmap
{
public:
  /**
   * @brief  Constructor for a costmap
   */
  LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

  /**
   * @brief  Destructor
   */
  ~LayeredCostmap();

  /**
   * @brief  Update the underlying costmap with new data.
   * If you want to update the map outside of the update loop that runs, you can call this.
   */
  void updateMap(double robot_x, double robot_y, double robot_yaw);

  std::string getGlobalFrameID() const { return global_frame_; }

  /**
   * @brief Resize the map to a new size, resolution, or origin
   */
  void resizeMap(
    unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
    bool size_locked = false);

  /**
   * @brief Get the size of the bounds for update
   */
  void getUpdatedBounds(double & minx, double & miny, double & maxx, double & maxy)
  {
    minx = minx_;
    miny = miny_;
    maxx = maxx_;
    maxy = maxy_;
  }

  /**
   * @brief If the costmap is current, e.g. are all the layers processing recent data
   * and not stale information for a good state.
   */
  bool isCurrent();

  /**
   * @brief Get the costmap pointer to the master costmap
   */
  Costmap2D * getCostmap() { return &combined_costmap_; }

  /**
   * @brief If this costmap is rolling or not
   */
  bool isRolling() { return rolling_window_; }

  /**
   * @brief If this costmap is tracking unknown space or not
   */
  bool isTrackingUnknown()
  {
    return combined_costmap_.getDefaultValue() == syncai_costmap::NO_INFORMATION;
  }

  /**
   * @brief Get the vector of pointers to the costmap plugins
   */
  std::vector<std::shared_ptr<Layer>> * getPlugins() { return &plugins_; }

  /**
   * @brief Add a new plugin to the plugins vector to process
   */
  void addPlugin(std::shared_ptr<Layer> plugin);

  /**
   * @brief Get if the size of the costmap is locked
   */
  bool isSizeLocked() { return size_locked_; }

  /**
   * @brief Get the bounds of the costmap
   */
  void getBounds(unsigned int * x0, unsigned int * xn, unsigned int * y0, unsigned int * yn)
  {
    *x0 = bx0_;
    *xn = bxn_;
    *y0 = by0_;
    *yn = byn_;
  }

  /**
   * @brief if the costmap is initialized
   */
  bool isInitialized() { return initialized_; }

  /** @brief Updates the stored footprint, updates the circumscribed
   * and inscribed radii, and calls onFootprintChanged() in all
   * layers. */
  void setFootprint(const std::vector<geometry_msgs::msg::Point> & footprint_spec);

  /** @brief Returns the latest footprint stored with setFootprint(). */
  const std::vector<geometry_msgs::msg::Point> & getFootprint() { return footprint_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which just surrounds all points on the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getCircumscribedRadius() { return circumscribed_radius_; }

  /** @brief The radius of a circle centered at the origin of the
   * robot which is just within all points and edges of the robot's
   * footprint.
   *
   * This is updated by setFootprint(). */
  double getInscribedRadius() { return inscribed_radius_; }

  /** @brief Checks if the robot is outside the bounds of its costmap in the case
  * of poorly configured setups. */
  bool isOutofBounds(double robot_x, double robot_y);

private:
  // One grid. Upstream keeps a primary_costmap_ beside it for its costmap
  // filters to run after the layers; the filter pipeline went with the 2D
  // keepout filter (2026-10-28), so the layers write the master directly.
  Costmap2D combined_costmap_;
  std::string global_frame_;

  bool rolling_window_;

  // Whether the last updateMap() found the robot outside the costmap. The
  // warning is logged on entering that state and an info on leaving it, not
  // every update (2026-10-19): a global costmap with no static map stays at
  // its 5 x 5 m default -- every update of a 3D-only map, where nothing
  // reads its cells -- and the per-update warning buried the planner log.
  bool robot_out_of_bounds_{false};

  bool current_;
  double minx_, miny_, maxx_, maxy_;
  unsigned int bx0_, bxn_, by0_, byn_;

  std::vector<std::shared_ptr<Layer>> plugins_;

  bool initialized_;
  bool size_locked_;
  double circumscribed_radius_, inscribed_radius_;
  std::vector<geometry_msgs::msg::Point> footprint_;
};

}  // namespace syncai_costmap

#endif