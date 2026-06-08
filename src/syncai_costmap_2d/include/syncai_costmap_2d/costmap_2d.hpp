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
   * @brief  Turn this costmap into a copy of a window of a costmap passed in
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
   * @brief Copies the (x0,y0)..(xn,yn) window from source costmap into a current costmap
     @param source Source costmap where the window will be copied from
     @param sx0 Lower x-boundary of the source window to copy, in cells
     @param sy0 Lower y-boundary of the source window to copy, in cells
     @param sxn Upper x-boundary of the source window to copy, in cells
     @param syn Upper y-boundary of the source window to copy, in cells
     @param dx0 Lower x-boundary of the destination window to copy, in cells
     @param dx0 Lower y-boundary of the destination window to copy, in cells
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
   * @brief Costmap 是一張 2D陣列圖，cell用(mx, my) 這種整數index存取；但 planner / TF / sensor 資料用的是 世界座標 (wx, wy)（單位：公尺）。
   *        mapToWorld 就是把兩個座標系統之間的轉換橋樑
   */
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;

  /**
   * @brief World座標轉成Map座標，輸入的世界座標(wx, wy)會被轉換成整數的地圖座標(mx, my)，並且存在mx, my裡面
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  /**
   * @brief World座標轉成Map座標，輸入的世界座標(wx, wy)會被轉換成浮點數的地圖座標(mx, my)，並且存在mx, my裡面
   *        保留Cell的小數部分。
   */
  bool worldToMapContinuous(double wx, double wy, float & mx, float & my) const;

  /**
   * @brief 不做邊界檢查，回傳int(可以是負值) 給內部已經確認邊界、追求極限速度的迴圈用。
   */
  void worldToMapNoBounds(double wx, double wy, int & mx, int & my) const;

  /**
    * @brief 邊界外的值會被 clip 到邊界內，不會失敗。給「找最近的合法 cell」這種需求
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

  /**
   * @brief Get costmap metadata: origin and resolution.
   */
  double getOriginX() const;
  double getOriginY() const;
  double getResolution() const;

  void setDefaultValue(unsigned char c) { default_value_ = c; }
  unsigned char getDefaultValue() { return default_value_; }

  /**
   * @brief 把「一個凸多邊形（在世界座標）區域內的所有cell，全部assign成指定的cost值」
   */
  bool setConvexPolygonCost(
    const std::vector<geometry_msgs::msg::Point> & polygon, unsigned char cost_value);

  void polygonOutlineCells(
    const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells);

  void convexFillCells(
    const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells);

  virtual void updateOrigin(double new_origin_x, double new_origin_y);

  /**
   * @brief 把當前costmap輸出成一份pgm檔案
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
    * @brief 刪除costmap, static map和所有markers裡面的資料.
    */
  virtual void deleteMaps();

  /**
   * @brief 重置costmap和static map裡面所有的cell值成unknown space.
   */
  virtual void resetMaps();

  /**
   * @brief 初始化整個costmap和static map跟所以markers裡面的資料.
   */
  virtual void initMaps(unsigned int size_x, unsigned int size_y);

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

  // MarkCell是一個functor(函式物件)，扮演「ray-tracing 走過每個 cell 時要做什麼動作」
  class MarkCell
  {
  public:
    MarkCell(unsigned char * costmap, unsigned char value) : costmap_(costmap), value_(value) {}

    inline void operator()(unsigned int offset) { costmap_[offset] = value_; }

  private:
    unsigned char * costmap_;
    unsigned char value_;
  };

  // PolygonOutlineCells 是一個「收集器」functor，作用在「在ray-trace走線時，把經過的每個 cell 座標收集到 vector 裡」
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
      //   costmap_.indexToCells(offset, loc.x, loc.y);
      cells_.push_back(loc);
    }

  private:
    const Costmap2D & costmap_;
    std::vector<MapLocation> & cells_;
  };
};

}  // namespace syncai_costmap_2d

#endif