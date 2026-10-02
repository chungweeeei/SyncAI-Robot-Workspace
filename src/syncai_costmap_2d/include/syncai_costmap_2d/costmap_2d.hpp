#ifndef SYNCAI_NAV2_COSTMAP_2D_COSTMAP_2D_HPP_
#define SYNCAI_NAV2_COSTMAP_2D_COSTMAP_2D_HPP_

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace syncai_costmap_2d
{
// convenient for storing x / y point pairs
struct MapLocation
{
  unsigned int x;
  unsigned int y;
};

/**
 * @class Costmap2D
 * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
 */
class Costmap2D
{
public:
  /**
   * @brief  Constructor for a costmap
   * @param  cells_size_x The x size of the map in cells
   * @param  cells_size_y The y size of the map in cells
   * @param  resolution The resolution of the map in meters/cell
   * @param  origin_x The x origin of the map
   * @param  origin_y The y origin of the map
   * @param  default_value Default Value
   */
  Costmap2D(
    unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x,
    double origin_y, unsigned char default_value = 0);

  /**
   * @brief  Copy constructor for a costmap, creates a copy efficiently
   * @param map The costmap to copy
   */
  Costmap2D(const Costmap2D & map);

  /**
   * @brief  Constructor for a costmap from an OccupancyGrid map
   * @param  map The OccupancyGrid map to create costmap from
   */
  explicit Costmap2D(const nav_msgs::msg::OccupancyGrid & map);

  /**
   * @brief  Overloaded assignment operator
   * @param  map The costmap to copy
   * @return A reference to the map after the copy has finished
   */
  Costmap2D & operator=(const Costmap2D & map);

  /**
   * @brief  Copy a rectangular "window" region out of another costmap so that this Costmap2D
   *         object becomes a copy of that window.
   * @param  map The costmap to copy
   * @param win_origin_x The x origin (lower left corner) for the window to copy, in meters
   * @param win_origin_y The y origin (lower left corner) for the window to copy, in meters
   * @param win_size_x The x size of the window, in meters
   * @param win_size_y The y size of the window, in meters
   */
  bool copyCostmapWindow(
    const Costmap2D & map, double win_origin_x, double win_origin_y, double win_size_x,
    double win_size_y);

  /**
   * @brief Copy the (x0,y0)..(xn,yn) window of the source costmap into this costmap
     @param source Source costmap
     @param sx0 Lower bound x of the source window, in cells
     @param sy0 Lower bound y of the source window, in cells
     @param sxn Upper bound x of the source window, in cells
     @param syn Upper bound y of the source window, in cells
     @param dx0 Lower bound x of the destination window, in cells
     @param dy0 Lower bound y of the destination window, in cells
     @returns true if copy was succeeded or false in negative case
   */
  bool copyWindow(
    const Costmap2D & source, unsigned int sx0, unsigned int sy0, unsigned int sxn,
    unsigned int syn, unsigned int dx0, unsigned int dy0);

  /**
   * @brief  Default constructor
   */
  Costmap2D();

  /**
   * @brief  Destructor
   */
  virtual ~Costmap2D();

  /**
   * @brief  Get the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @return The cost of the cell
   */
  unsigned char getCost(unsigned int mx, unsigned int my) const;

  /**
   * @brief  Get the cost of a cell in the costmap
   * @param index The cell index
   * @return The cost of the cell
   */
  unsigned char getCost(unsigned int index) const;

  /**
   * @brief  Set the cost of a cell in the costmap
   * @param mx The x coordinate of the cell
   * @param my The y coordinate of the cell
   * @param cost The cost to set the cell to
   */
  void setCost(unsigned int mx, unsigned int my, unsigned char cost);

  /**
   * @brief The costmap is a 2D array whose cells are addressed by integer indices (mx, my),
   *        but planner / TF / sensor data use world coordinates (wx, wy), in meters.
   *        mapToWorld is the bridge that converts between the two coordinate systems.
   */
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  /**
   * @brief World to map coordinates: the input world coordinates (wx, wy) are converted to
   *        integer map coordinates (mx, my) and stored in mx, my.
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  /**
   * @brief World to map coordinates: the input world coordinates (wx, wy) are converted to
   *        floating-point map coordinates (mx, my) and stored in mx, my, keeping the
   *        fractional part of the cell.
   */
  bool worldToMapContinuous(double wx, double wy, float & mx, float & my) const;

  /**
   * @brief No bounds checking; returns int (may be negative). For internal loops that have
   *        already verified the bounds and want maximum speed.
   */
  void worldToMapNoBounds(double wx, double wy, int & mx, int & my) const;

  /**
    * @brief Values outside the map are clipped to the boundary, so it never fails. For needs
    *        like "find the nearest valid cell".
    */
  void worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const;

  // Given two map coordinates... compute the associated index
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const { return my * size_x_ + mx; }

  // Given an index... compute the associated map coordinates
  inline void indexToCells(unsigned int index, unsigned int & mx, unsigned int & my) const
  {
    my = index / size_x_;
    mx = index - (my * size_x_);
  }

  /**
   * @brief Return a pointer to the underlying unsigned char array used as the costmap.
   */
  unsigned char * getCharMap() const;

  unsigned int getSizeInCellsX() const;
  unsigned int getSizeInCellsY() const;
  double getSizeInMetersX() const;
  double getSizeInMetersY() const;

  double getOriginX() const;
  double getOriginY() const;
  double getResolution() const;

  void setDefaultValue(unsigned char c) { default_value_ = c; }
  unsigned char getDefaultValue() { return default_value_; }

  /**
   * @brief Assign the given cost value to every cell inside a convex polygon (given in world
   *        coordinates).
   */
  bool setConvexPolygonCost(
    const std::vector<geometry_msgs::msg::Point> & polygon, unsigned char cost_value);

  void polygonOutlineCells(
    const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells);

  void convexFillCells(
    const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells);

  virtual void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief Write the current costmap out as a pgm file
   */
  bool saveMap(std::string file_name);

  void resizeMap(
    unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y);

  void resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn);

  void resetMapToValue(
    unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value);

  unsigned int cellDistance(double world_dist);

  // Provide a typedef to ease future code maintenance
  typedef std::recursive_mutex mutex_t;
  mutex_t * getMutex() { return access_; }

protected:
  /**
   * @brief  Copy a region of a source map into a destination map
   * @param  source_map The source map
   * @param sm_lower_left_x The lower left x point of the source map to start the copy
   * @param sm_lower_left_y The lower left y point of the source map to start the copy
   * @param sm_size_x The x size of the source map
   * @param  dest_map The destination map
   * @param dm_lower_left_x The lower left x point of the destination map to start the copy
   * @param dm_lower_left_y The lower left y point of the destination map to start the copy
   * @param dm_size_x The x size of the destination map
   * @param region_size_x The x size of the region to copy
   * @param region_size_y The y size of the region to copy
   */
  template <typename data_type>
  void copyMapRegion(
    data_type * source_map, unsigned int sm_lower_left_x, unsigned int sm_lower_left_y,
    unsigned int sm_size_x, data_type * dest_map, unsigned int dm_lower_left_x,
    unsigned int dm_lower_left_y, unsigned int dm_size_x, unsigned int region_size_x,
    unsigned int region_size_y)
  {
    // we'll first need to compute the starting points for each map
    data_type * sm_index = source_map + (sm_lower_left_y * sm_size_x + sm_lower_left_x);
    data_type * dm_index = dest_map + (dm_lower_left_y * dm_size_x + dm_lower_left_x);

    // now, we'll copy the source map into the destination map
    for (unsigned int i = 0; i < region_size_y; ++i) {
      memcpy(dm_index, sm_index, region_size_x * sizeof(data_type));
      sm_index += sm_size_x;
      dm_index += dm_size_x;
    }
  }
  /**
    * @brief Delete the data of the costmap, the static map and all markers.
    */
  virtual void deleteMaps();

  /**
   * @brief Reset every cell of the costmap and the static map to unknown space.
   */
  virtual void resetMaps();

  /**
   * @brief Initialize the data of the whole costmap, the static map and all markers.
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

  // TODO: know what is raytraceLine
  template <class ActionType>
  inline void raytraceLine(
    ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,
    unsigned int max_length = UINT_MAX, unsigned int min_length = 0)
  {
    int dx_full = x1 - x0;
    int dy_full = y1 - y0;

    // we need to chose how much to scale our dominant dimension,
    // based on the maximum length of the line
    double dist = std::hypot(dx_full, dy_full);
    if (dist < min_length) {
      return;
    }

    unsigned int min_x0, min_y0;
    if (dist > 0.0) {
      // Adjust starting point and offset to start from min_length distance
      min_x0 = (unsigned int)(x0 + dx_full / dist * min_length);
      min_y0 = (unsigned int)(y0 + dy_full / dist * min_length);
    } else {
      // dist can be 0 if [x0, y0]==[x1, y1].
      // In this case only this cell should be processed.
      min_x0 = x0;
      min_y0 = y0;
    }
    unsigned int offset = min_y0 * size_x_ + min_x0;

    int dx = x1 - min_x0;
    int dy = y1 - min_y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;

    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);
    // if x is dominant
    if (abs_dx >= abs_dy) {
      int error_y = abs_dx / 2;

      bresenham2D(
        at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
      return;
    }

    // otherwise y is dominant
    int error_x = abs_dy / 2;

    bresenham2D(
      at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
  }

private:
  /**
   * @brief  A 2D implementation of Bresenham's raytracing algorithm...
   * applies an action at each step
   */
  template <class ActionType>
  inline void bresenham2D(
    ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
    int offset_b, unsigned int offset, unsigned int max_length)
  {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i) {
      at(offset);
      offset += offset_a;
      error_b += abs_db;
      if ((unsigned int)error_b >= abs_da) {
        offset += offset_b;
        error_b -= abs_da;
      }
    }
    at(offset);
  }

  /**
   * @brief get the sign of an int
   */
  inline int sign(int x) { return x > 0 ? 1.0 : -1.0; }

  mutex_t * access_;

protected:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;

  unsigned char * costmap_;
  unsigned char default_value_;

  // MarkCell is a functor (function object) that plays the role of "what to do at each cell
  // the ray-tracing walks over"
  class MarkCell
  {
  public:
    MarkCell(unsigned char * costmap, unsigned char value) : costmap_(costmap), value_(value) {}

    inline void operator()(unsigned int offset) { costmap_[offset] = value_; }

  private:
    unsigned char * costmap_;
    unsigned char value_;
  };

  // PolygonOutlineCells is a "collector" functor: while ray-tracing a line, it collects the
  // coordinates of every cell it passes through into a vector
  class PolygonOutlineCells
  {
  public:
    PolygonOutlineCells(
      const Costmap2D & costmap, const unsigned char * /*char_map*/,
      std::vector<MapLocation> & cells)
    : costmap_(costmap), cells_(cells)
    {
    }

    inline void operator()(unsigned int offset)
    {
      MapLocation loc;
      costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const Costmap2D & costmap_;
    std::vector<MapLocation> & cells_;
  };
};

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D_COSTMAP_2D_HPP_