#ifndef SYNCAI_MAP_SERVER__MAP_MODE_HPP_
#define SYNCAI_MAP_SERVER__MAP_MODE_HPP_

#include <string>
#include <vector>

namespace syncai_map_server
{
/**
 * Describes the relation between image pixel values and map occupancy status (0-100; -1).
 * Lightness refers to the mean of a given pixel's RGB channels on a scale from 0 to 255.
*/
enum class MapMode { Trinary, Scale, Raw };

const char * map_mode_to_string(MapMode map_mode);

MapMode map_mode_from_string(std::string map_mode_name);
}  // namespace syncai_map_server

#endif  // SYNCAI_MAP_SERVER__MAP_MODE_HPP_