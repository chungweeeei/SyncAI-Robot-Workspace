#ifndef SYNCAI_MAP_SERVER__MAP_IO_HPP_
#define SYNCAI_MAP_SERVER__MAP_IO_HPP_

#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "syncai_map_server/map_mode.hpp"

namespace syncai_map_server
{

struct LoadParameters
{
  // map metadata
  std::string image_file_name;
  double resolution{0.0};
  std::vector<double> origin{0, 0, 0};
  double free_thresh;
  double occupied_thresh;
  MapMode mode;
  bool negate;
};

typedef enum {
  LOAD_MAP_SUCCESS,
  MAP_DOES_NOT_EXIST,
  INVALID_MAP_METADATA,
  INVALID_MAP_DATA
} LOAD_MAP_STATUS;

LoadParameters loadMapYaml(const std::string & yaml_filename);

void loadMapFromFile(const LoadParameters & load_parameters, nav_msgs::msg::OccupancyGrid & map);

LOAD_MAP_STATUS loadMapFromYaml(const std::string & yaml_file, nav_msgs::msg::OccupancyGrid & map);

/*
    Map output part
*/

struct SaveParameters
{
  std::string map_file_name{""};
  std::string image_format{""};
  double free_thresh{0.0};
  double occupied_thresh{0.0};
  MapMode mode{MapMode::Trinary};
};

bool saveMapToFile(
  const nav_msgs::msg::OccupancyGrid & map, const SaveParameters & save_parameters);

std::string to_string_with_precision(double value, int precision);

std::string expand_user_home_dir_if_needed(std::string yaml_filename, std::string home_dir);

}  // namespace syncai_map_server

#endif  // SYNCAI_MAP_SERVER__MAP_IO_HPP_