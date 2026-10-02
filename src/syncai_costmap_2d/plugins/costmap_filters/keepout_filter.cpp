#include "syncai_costmap_2d/costmap_filters/keepout_filter.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "syncai_costmap_2d/cost_values.hpp"
#include "syncai_costmap_2d/costmap_filters/filter_values.hpp"
#include "syncai_costmap_2d/inflation_layer.hpp"
#include "syncai_costmap_2d/layered_costmap.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(syncai_costmap_2d::KeepoutFilter, syncai_costmap_2d::Layer)

namespace syncai_costmap_2d
{

KeepoutFilter::KeepoutFilter()
: filter_info_sub_(nullptr),
  mask_sub_(nullptr),
  mask_costmap_(nullptr),
  inflated_mask_(nullptr),
  mask_frame_(""),
  global_frame_("")
{
}

void KeepoutFilter::initializeFilter(const std::string & filter_info_topic)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!node_) {
    throw std::runtime_error{"Failed to get node in KeepoutFilter"};
  }

  filter_info_topic_ = filter_info_topic;
  // Setting new costmap filter info subscriber
  RCLCPP_INFO(
    logger_, "[KeepoutFilter][%s] Subscribing to \"%s\" topic for filter info...", __func__,
    filter_info_topic_.c_str());
  filter_info_sub_ = node_->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::filterInfoCallback, this, std::placeholders::_1));

  global_frame_ = layered_costmap_->getGlobalFrameID();
}

void KeepoutFilter::filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!node_) {
    throw std::runtime_error{"Failed to get node in KeepoutFilter"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_, "[KeepoutFilter][%s] Received filter info from %s topic.", __func__,
      filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "[KeepoutFilter][%s] New costmap filter info arrived from %s topic. "
      "Updating old filter info.",
      __func__, filter_info_topic_.c_str());
    // Resetting previous subscriber each time when new costmap filter information arrives
    mask_sub_.reset();
  }

  // Checking that base and multiplier are set to their default values
  if (msg->base != BASE_DEFAULT || msg->multiplier != MULTIPLIER_DEFAULT) {
    RCLCPP_ERROR(
      logger_,
      "[KeepoutFilter][%s] For proper use of keepout filter base and multiplier"
      " in CostmapFilterInfo message should be set to their default values (%f and %f)",
      __func__, BASE_DEFAULT, MULTIPLIER_DEFAULT);
  }

  // The costmap node lives in its own sub-namespace; resolve a relative mask
  // topic against the PARENT namespace so it still points at /<parent>/<mask_topic>
  // (the mask map server publishes in the parent namespace, like map_server).
  mask_topic_ = joinWithParentNamespace(msg->filter_mask_topic);

  // Setting new filter mask subscriber
  RCLCPP_INFO(
    logger_, "[KeepoutFilter][%s] Subscribing to \"%s\" topic for filter mask...", __func__,
    mask_topic_.c_str());
  mask_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&KeepoutFilter::maskCallback, this, std::placeholders::_1));
}

void KeepoutFilter::maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_costmap_) {
    RCLCPP_INFO(
      logger_, "[KeepoutFilter][%s] Received filter mask from %s topic.", __func__,
      mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_, "[KeepoutFilter][%s] New filter mask arrived from %s topic. Updating old filter mask.",
      __func__, mask_topic_.c_str());
    mask_costmap_.reset();
    inflated_mask_.reset();
  }

  // Making a new mask_costmap_
  mask_costmap_ = std::make_unique<Costmap2D>(*msg);
  mask_frame_ = msg->header.frame_id;

  inflateMask();
}

void KeepoutFilter::onFootprintChanged()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  // Costmap2DROS sets the footprint during construction, before any mask has
  // arrived; the mask callback inflates with whatever radius is current then.
  if (!mask_costmap_) {
    return;
  }
  inflateMask();
}

void KeepoutFilter::inflateMask()
{
  // Radii come from the costmap this filter is attached to, not from
  // parameters of its own: one robot, one footprint, and the zone has to
  // carry the same band and gradient the InflationLayer puts around a wall or
  // the planner treats the two differently. The inscribed radius is derived
  // from the padded footprint by LayeredCostmap::setFootprint(); the radius
  // and decay are read off the InflationLayer the same way
  // syncai_planner's findCircumscribedCost() finds it.
  const double inscribed_radius = layered_costmap_->getInscribedRadius();
  double inflation_radius = inscribed_radius;
  double cost_scaling_factor = 0.0;
  bool inflation_layer_found = false;
  for (const auto & plugin : *layered_costmap_->getPlugins()) {
    auto inflation_layer = std::dynamic_pointer_cast<InflationLayer>(plugin);
    if (!inflation_layer) {
      continue;
    }
    inflation_layer_found = true;
    inflation_radius = inflation_layer->getInflationRadius();
    cost_scaling_factor = inflation_layer->getCostScalingFactor();
  }

  if (!inflation_layer_found) {
    // Without an InflationLayer the walls have no band either, so match them:
    // only the INSCRIBED disc that closes gaps narrower than the robot.
    RCLCPP_WARN(
      logger_,
      "[KeepoutFilter][%s] No inflation layer in this costmap; keepout zones get the inscribed "
      "band (%.2f m) only, no cost gradient",
      __func__, inscribed_radius);
  } else if (inflation_radius < inscribed_radius) {
    // Mirrors InflationLayer exactly, which also stops at inflation_radius —
    // but then the band is too thin for centre-cell collision checking to
    // keep the footprint clear, for zones and walls alike.
    RCLCPP_WARN(
      logger_,
      "[KeepoutFilter][%s] inflation_radius %.2f m < inscribed radius %.2f m: zones (and walls) "
      "are inflated less than the robot's half-width",
      __func__, inflation_radius, inscribed_radius);
  }

  const double resolution = mask_costmap_->getResolution();
  const unsigned int size_x = mask_costmap_->getSizeInCellsX();
  const unsigned int size_y = mask_costmap_->getSizeInCellsY();

  // Same curve as InflationLayer::computeCost(), but in metres rather than
  // in cells: the mask is served by its own map_server and its resolution is
  // not guaranteed to be the costmap's.
  auto cost_at = [&](double distance) -> unsigned char {
      if (distance <= 0.0) {
        return LETHAL_OBSTACLE;
      }
      if (distance <= inscribed_radius) {
        return INSCRIBED_INFLATED_OBSTACLE;
      }
      const double factor = std::exp(-cost_scaling_factor * (distance - inscribed_radius));
      return static_cast<unsigned char>((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    };

  // Disc kernel of cell offsets with their cost, precomputed once; stamped
  // (max) around every boundary cell of every zone. Interior cells are
  // skipped: everything their disc would reach, a boundary cell's disc
  // reaches too. Boundary = LETHAL with a 4-neighbour that is not LETHAL,
  // the map edge counting as not-LETHAL (conservative, and cheap).
  struct KernelCell
  {
    int dx;
    int dy;
    unsigned char cost;
  };
  const int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution));
  std::vector<KernelCell> kernel;
  kernel.reserve(static_cast<size_t>((2 * radius_cells + 1) * (2 * radius_cells + 1)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double distance = std::hypot(dx, dy) * resolution;
      if (distance > inflation_radius) {
        continue;
      }
      const unsigned char cost = cost_at(distance);
      if (cost == FREE_SPACE) {
        continue;
      }
      kernel.push_back({dx, dy, cost});
    }
  }

  inflated_mask_ = std::make_unique<Costmap2D>(*mask_costmap_);
  const unsigned char * raw = mask_costmap_->getCharMap();
  unsigned char * inflated = inflated_mask_->getCharMap();

  auto is_lethal = [&](int x, int y) -> bool {
      if (x < 0 || y < 0 || x >= static_cast<int>(size_x) || y >= static_cast<int>(size_y)) {
        return false;
      }
      return raw[mask_costmap_->getIndex(x, y)] == LETHAL_OBSTACLE;
    };

  size_t boundary_cells = 0;
  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      if (raw[mask_costmap_->getIndex(x, y)] != LETHAL_OBSTACLE) {
        continue;
      }
      const int ix = static_cast<int>(x);
      const int iy = static_cast<int>(y);
      if (
        is_lethal(ix - 1, iy) && is_lethal(ix + 1, iy) && is_lethal(ix, iy - 1) &&
        is_lethal(ix, iy + 1))
      {
        continue;  // interior
      }
      ++boundary_cells;
      for (const auto & k : kernel) {
        const int tx = ix + k.dx;
        const int ty = iy + k.dy;
        if (tx < 0 || ty < 0 || tx >= static_cast<int>(size_x) || ty >= static_cast<int>(size_y)) {
          continue;
        }
        const unsigned int index = mask_costmap_->getIndex(tx, ty);
        if (raw[index] == LETHAL_OBSTACLE) {
          continue;  // a drawn cell stays exactly as drawn
        }
        // An untouched mask cell may be NO_INFORMATION (255) — the blank mask
        // the nav session writes is all unknown — which would win a plain
        // max. Treat unknown as 0 here; process() tells drawn cells from
        // inflated ones by comparing against the raw mask anyway.
        const unsigned char current = (inflated[index] == NO_INFORMATION) ? 0 : inflated[index];
        inflated[index] = std::max(current, k.cost);
      }
    }
  }

  RCLCPP_INFO(
    logger_,
    "[KeepoutFilter][%s] Inflated %zu zone boundary cells: inscribed %.2f m, radius %.2f m, "
    "scaling %.1f (%s)",
    __func__, boundary_cells, inscribed_radius, inflation_radius, cost_scaling_factor,
    inflation_layer_found ? "from inflation_layer" : "no inflation_layer");

  has_updated_data_ = true;
}

void KeepoutFilter::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  CostmapFilter::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

  if (!has_updated_data_) {
    return;
  }

  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());
  if (!inflated_mask_) {
    has_updated_data_ = false;
    return;
  }

  // Grow the window to cover the whole mask once, so the new mask (or the
  // re-inflated one after a footprint change) is written everywhere, not
  // only inside the layers' bounds. Upstream converted the mask's cell count
  // through the *costmap's* mapToWorld, which is only right when mask and
  // costmap share origin and resolution; the mask's own extents are what is
  // meant. In another frame the mask's extents are not axis-aligned here, so
  // fall back to the whole costmap — a one-off cost.
  double mask_min_x, mask_min_y, mask_max_x, mask_max_y;
  if (mask_frame_ == global_frame_) {
    mask_min_x = inflated_mask_->getOriginX();
    mask_min_y = inflated_mask_->getOriginY();
    mask_max_x = mask_min_x + inflated_mask_->getSizeInMetersX();
    mask_max_y = mask_min_y + inflated_mask_->getSizeInMetersY();
  } else {
    Costmap2D * master = layered_costmap_->getCostmap();
    mask_min_x = master->getOriginX();
    mask_min_y = master->getOriginY();
    mask_max_x = mask_min_x + master->getSizeInMetersX();
    mask_max_y = mask_min_y + master->getSizeInMetersY();
  }

  *min_x = std::min(mask_min_x, *min_x);
  *min_y = std::min(mask_min_y, *min_y);
  *max_x = std::max(mask_max_x, *max_x);
  *max_y = std::max(mask_max_y, *max_y);

  has_updated_data_ = false;
}

void KeepoutFilter::process(
  Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j,
  const geometry_msgs::msg::Pose2D & /*pose*/)
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (!mask_costmap_ || !inflated_mask_) {
    // Show warning message every 2 seconds to not litter an output
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000, "[KeepoutFilter][%s] Filter mask was not received", __func__);
    return;
  }

  tf2::Transform tf2_transform;
  tf2_transform.setIdentity();  // initialize by identical transform
  int mg_min_x, mg_min_y;       // master_grid indexes of bottom-left window corner
  int mg_max_x, mg_max_y;       // master_grid indexes of top-right window corner

  if (mask_frame_ != global_frame_) {
    // Filter mask and current layer are in different frames:
    // prepare frame transformation if mask_frame_ != global_frame_
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        mask_frame_, global_frame_, tf2::TimePointZero, transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger_,
        "[KeepoutFilter][%s] Failed to get costmap frame (%s) "
        "transformation to mask frame (%s) with error: %s",
        __func__, global_frame_.c_str(), mask_frame_.c_str(), ex.what());
      return;
    }
    tf2::fromMsg(transform.transform, tf2_transform);

    mg_min_x = min_i;
    mg_min_y = min_j;
    mg_max_x = max_i;
    mg_max_y = max_j;
  } else {
    // Filter mask and current layer are in the same frame:
    // apply the following optimization - iterate only in overlapped
    // (min_i, min_j)..(max_i, max_j) & mask_costmap_ area.
    //
    //           mask_costmap_
    //       *----------------------------*
    //       |                            |
    //       |                            |
    //       |      (2)                   |
    // *-----+-------*                    |
    // |     |///////|<- overlapped area  |
    // |     |///////|   to iterate in    |
    // |     *-------+--------------------*
    // |    (1)      |
    // |             |
    // *-------------*
    //  master_grid (min_i, min_j)..(max_i, max_j) window
    //
    // ToDo: after costmap rotation will be added, this should be re-worked.

    double wx, wy;  // world coordinates

    // Calculating bounds corresponding to bottom-left overlapping (1) corner
    // mask_costmap_ -> master_grid indexes conversion
    const double half_cell_size = 0.5 * mask_costmap_->getResolution();
    wx = mask_costmap_->getOriginX() + half_cell_size;
    wy = mask_costmap_->getOriginY() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_min_x, mg_min_y);
    // Calculation of (1) corner bounds
    if (mg_min_x >= max_i || mg_min_y >= max_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_min_x = std::max(min_i, mg_min_x);
    mg_min_y = std::max(min_j, mg_min_y);

    // Calculating bounds corresponding to top-right window (2) corner
    // mask_costmap_ -> master_grid indexes conversion
    wx = mask_costmap_->getOriginX() +
         mask_costmap_->getSizeInCellsX() * mask_costmap_->getResolution() + half_cell_size;
    wy = mask_costmap_->getOriginY() +
         mask_costmap_->getSizeInCellsY() * mask_costmap_->getResolution() + half_cell_size;
    master_grid.worldToMapNoBounds(wx, wy, mg_max_x, mg_max_y);
    // Calculation of (2) corner bounds
    if (mg_max_x <= min_i || mg_max_y <= min_j) {
      // There is no overlapping. Do nothing.
      return;
    }
    mg_max_x = std::min(max_i, mg_max_x);
    mg_max_y = std::min(max_j, mg_max_y);
  }

  // unsigned<-signed conversions.
  unsigned const int mg_min_x_u = static_cast<unsigned int>(mg_min_x);
  unsigned const int mg_min_y_u = static_cast<unsigned int>(mg_min_y);
  unsigned const int mg_max_x_u = static_cast<unsigned int>(mg_max_x);
  unsigned const int mg_max_y_u = static_cast<unsigned int>(mg_max_y);

  unsigned int i, j;              // master_grid iterators
  unsigned int index;             // corresponding index of master_grid
  double gl_wx, gl_wy;            // world coordinates in a global_frame_
  double msk_wx, msk_wy;          // world coordinates in a mask_frame_
  unsigned int mx, my;            // mask_costmap_ coordinates
  unsigned char data, raw, old_data;  // inflated mask / drawn mask / master_grid element data

  // Main master_grid updating loop
  // Iterate in costmap window by master_grid indexes
  unsigned char * master_array = master_grid.getCharMap();
  for (i = mg_min_x_u; i < mg_max_x_u; i++) {
    for (j = mg_min_y_u; j < mg_max_y_u; j++) {
      index = master_grid.getIndex(i, j);
      old_data = master_array[index];
      // Calculating corresponding to (i, j) point at mask_costmap_:
      // Get world coordinates in global_frame_
      master_grid.mapToWorld(i, j, gl_wx, gl_wy);
      if (mask_frame_ != global_frame_) {
        // Transform (i, j) point from global_frame_ to mask_frame_
        tf2::Vector3 point(gl_wx, gl_wy, 0);
        point = tf2_transform * point;
        msk_wx = point.x();
        msk_wy = point.y();
      } else {
        // In this case master_grid and filter-mask are in the same frame
        msk_wx = gl_wx;
        msk_wy = gl_wy;
      }
      // Get mask coordinates corresponding to (i, j) point at mask_costmap_
      if (inflated_mask_->worldToMap(msk_wx, msk_wy, mx, my)) {
        data = inflated_mask_->getCost(mx, my);
        if (data == NO_INFORMATION) {
          continue;  // unknown mask cell, untouched by any zone's inflation
        }
        raw = mask_costmap_->getCost(mx, my);
        if (raw == NO_INFORMATION || raw != data) {
          // Produced by inflation, not drawn. It is a derived cost, not a
          // rule about the cell, so it must never turn an unexplored master
          // cell into known, plannable space: skip unknown, max otherwise.
          if (old_data == NO_INFORMATION) {
            continue;
          }
          if (data > old_data) {
            master_array[index] = data;
          }
        } else if (data > old_data || old_data == NO_INFORMATION) {
          // Drawn cell: upstream semantics. A free mask cell overwrites an
          // unknown master cell, which is why the blank mask is all unknown.
          master_array[index] = data;
        }
      }
    }
  }
}

void KeepoutFilter::resetFilter()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
}

bool KeepoutFilter::isActive()
{
  std::lock_guard<CostmapFilter::mutex_t> guard(*getMutex());

  if (mask_costmap_) {
    return true;
  }
  return false;
}

}  // namespace syncai_costmap_2d
