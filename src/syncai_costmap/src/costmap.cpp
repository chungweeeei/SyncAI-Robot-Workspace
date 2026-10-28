#include "syncai_costmap/costmap.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "syncai_costmap/cost_values.hpp"

namespace syncai_costmap
{
Costmap::Costmap(
  unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x,
  double origin_y, unsigned char default_value)
: resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  costmap_(NULL),
  default_value_(default_value)
{
  access_ = new mutex_t();

  // create the costmap
  initMaps(cells_size_x, cells_size_y);
  resetMaps();
};

void Costmap::deleteMaps()
{
  // clean up data
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  costmap_ = NULL;
}

void Costmap::initMaps(unsigned int size_x, unsigned int size_y)
{
  std::unique_lock<mutex_t> lock(*access_);
  delete[] costmap_;
  size_x_ = size_x;
  size_y_ = size_y;
  costmap_ = new unsigned char[size_x_ * size_y_];
};

void Costmap::resizeMap(
  unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y)
{
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  initMaps(size_x, size_y);

  // reset our maps to have no information
  resetMaps();
}

void Costmap::resetMaps()
{
  std::unique_lock<mutex_t> lock(*access_);

  // memset => set every byte of the memory region to the same assigned value
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

void Costmap::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  resetMapToValue(x0, y0, xn, yn, default_value_);
}

void Costmap::resetMapToValue(
  unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn, unsigned char value)
{
  std::unique_lock<mutex_t> lock(*access_);

  unsigned int len = xn - x0;
  for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_) {
    memset(costmap_ + y, value, len * sizeof(unsigned char));
  }
}

bool Costmap::copyCostmapWindow(
  const Costmap & map, double win_origin_x, double win_origin_y, double win_size_x,
  double win_size_y)
{
  if (this == &map) {
    return false;
  }

  // cleanup old data
  deleteMaps();

  // compute the bounds of our new map
  unsigned int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  if (
    !map.worldToMap(win_origin_x, win_origin_y, lower_left_x, lower_left_y) ||
    !map.worldToMap(
      win_origin_x + win_size_x, win_origin_y + win_size_y, upper_right_x, upper_right_y)) {
    // ROS_ERROR("Cannot window a map that the window bounds don't fit inside of");
    return false;
  }

  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;

  // initialize our various maps and reset markers for inflation
  initMaps(upper_right_x - lower_left_x, upper_right_y - lower_left_y);

  // copy the window for static map and the csotmap that we're taking
  copyMapRegion(
    map.costmap_, lower_left_x, lower_left_y, map.size_x_, costmap_, 0, 0, size_x_, size_x_,
    size_y_);

  return true;
}

Costmap & Costmap::operator=(const Costmap & map)
{
  if (this == &map) {
    return *this;
  }

  // cleanup old data
  deleteMaps();

  size_x_ = map.size_x_;
  size_y_ = map.size_y_;
  resolution_ = map.resolution_;
  origin_x_ = map.origin_x_;
  origin_y_ = map.origin_y_;
  default_value_ = map.default_value_;

  // initialize out various maps
  initMaps(size_x_, size_y_);

  // copy the cost map
  memcpy(costmap_, map.costmap_, size_x_ * size_y_ * sizeof(unsigned char));

  return *this;
}

Costmap::Costmap(const Costmap & map) : costmap_(NULL)
{
  access_ = new mutex_t();
  *this = map;
}

Costmap::Costmap()
: size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0), costmap_(NULL)
{
  access_ = new mutex_t();
}

Costmap::~Costmap()
{
  deleteMaps();
  delete access_;
}

unsigned int Costmap::cellDistance(double world_dist)
{
  double cells_dist = std::max(0.0, ceil(world_dist / resolution_));
  return (unsigned int)cells_dist;
}

unsigned char * Costmap::getCharMap() const
{
  return costmap_;
}

unsigned char Costmap::getCost(unsigned int mx, unsigned int my) const
{
  return costmap_[getIndex(mx, my)];
}

unsigned char Costmap::getCost(unsigned int undex) const
{
  return costmap_[undex];
}

void Costmap::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
  costmap_[getIndex(mx, my)] = cost;
}

void Costmap::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

bool Costmap::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

bool Costmap::worldToMapContinuous(double wx, double wy, float & mx, float & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  mx = static_cast<float>((wx - origin_x_) / resolution_);
  my = static_cast<float>((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}

void Costmap::worldToMapNoBounds(double wx, double wy, int & mx, int & my) const
{
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);
}

void Costmap::worldToMapEnforceBounds(double wx, double wy, int & mx, int & my) const
{
  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_) {
    mx = 0;
  } else if (wx > resolution_ * size_x_ + origin_x_) {
    mx = size_x_ - 1;
  } else {
    mx = static_cast<int>((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_) {
    my = 0;
  } else if (wy > resolution_ * size_y_ + origin_y_) {
    my = size_y_ - 1;
  } else {
    my = static_cast<int>((wy - origin_y_) / resolution_);
  }
}

void Costmap::updateOrigin(double new_origin_x, double new_origin_y)
{
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  int size_x = size_x_;
  int size_y = size_y_;

  // we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x,
    cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  // make sure to clean up
  delete[] local_map;
}

bool Costmap::setConvexPolygonCost(
  const std::vector<geometry_msgs::msg::Point> & polygon, unsigned char cost_value)
{
  std::vector<MapLocation> map_polygon;
  for (unsigned int i = 0; i < polygon.size(); ++i) {
    MapLocation loc;
    if (!worldToMap(polygon[i].x, polygon[i].y, loc.x, loc.y)) {
      // ("Polygon lies outside map bounds, so we can't fill it");
      return false;
    }
    map_polygon.push_back(loc);
  }

  std::vector<MapLocation> polygon_cells;

  // get the cells that fill the polygon
  convexFillCells(map_polygon, polygon_cells);

  // set the cost of those cells
  for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
    unsigned int index = getIndex(polygon_cells[i].x, polygon_cells[i].y);
    costmap_[index] = cost_value;
  }
  return true;
}

void Costmap::polygonOutlineCells(
  const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells)
{
  PolygonOutlineCells cell_gatherer(*this, costmap_, polygon_cells);
  for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
    raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
  }
  if (!polygon.empty()) {
    unsigned int last_index = polygon.size() - 1;
    // we also need to close the polygon by going from the last point to the first
    raytraceLine(
      cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
  }
}

void Costmap::convexFillCells(
  const std::vector<MapLocation> & polygon, std::vector<MapLocation> & polygon_cells)
{
  if (polygon.size() < 3) {
    return;
  }

  // first get the cells that make up the outline of the polygon
  polygonOutlineCells(polygon, polygon_cells);

  // quick bubble sort to sort points by x
  MapLocation swap;
  unsigned int i = 0;
  while (i < polygon_cells.size() - 1) {
    if (polygon_cells[i].x > polygon_cells[i + 1].x) {
      swap = polygon_cells[i];
      polygon_cells[i] = polygon_cells[i + 1];
      polygon_cells[i + 1] = swap;

      if (i > 0) {
        --i;
      }
    } else {
      ++i;
    }
  }

  i = 0;
  MapLocation min_pt;
  MapLocation max_pt;
  unsigned int min_x = polygon_cells[0].x;
  unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

  // walk through each column and mark cells inside the polygon
  for (unsigned int x = min_x; x <= max_x; ++x) {
    if (i >= polygon_cells.size() - 1) {
      break;
    }

    if (polygon_cells[i].y < polygon_cells[i + 1].y) {
      min_pt = polygon_cells[i];
      max_pt = polygon_cells[i + 1];
    } else {
      min_pt = polygon_cells[i + 1];
      max_pt = polygon_cells[i];
    }

    i += 2;
    while (i < polygon_cells.size() && polygon_cells[i].x == x) {
      if (polygon_cells[i].y < min_pt.y) {
        min_pt = polygon_cells[i];
      } else if (polygon_cells[i].y > max_pt.y) {
        max_pt = polygon_cells[i];
      }
      ++i;
    }

    MapLocation pt;
    // loop though cells in the column
    for (unsigned int y = min_pt.y; y <= max_pt.y; ++y) {
      pt.x = x;
      pt.y = y;
      polygon_cells.push_back(pt);
    }
  }
}

unsigned int Costmap::getSizeInCellsX() const
{
  return size_x_;
}

unsigned int Costmap::getSizeInCellsY() const
{
  return size_y_;
}

double Costmap::getSizeInMetersX() const
{
  return (size_x_ - 1 + 0.5) * resolution_;
}

double Costmap::getSizeInMetersY() const
{
  return (size_y_ - 1 + 0.5) * resolution_;
}

double Costmap::getOriginX() const
{
  return origin_x_;
}

double Costmap::getOriginY() const
{
  return origin_y_;
}

double Costmap::getResolution() const
{
  return resolution_;
}

bool Costmap::saveMap(std::string file_name)
{
  FILE * fp = fopen(file_name.c_str(), "w");

  if (!fp) {
    return false;
  }

  fprintf(fp, "P2\n%u\n%u\n%u\n", size_x_, size_y_, 0xff);
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      unsigned char cost = getCost(ix, iy);
      fprintf(fp, "%d ", cost);
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  return true;
}

}  // namespace syncai_costmap