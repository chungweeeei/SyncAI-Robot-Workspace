#ifndef SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_
#define SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_

#include <cstdint>

/** Provides constants used in costmap filters */

namespace syncai_costmap_2d
{

/** Types of costmap filter */
static constexpr uint8_t KEEPOUT_FILTER = 0;
static constexpr uint8_t SPEED_FILTER_PERCENT = 1;
static constexpr uint8_t SPEED_FILTER_ABSOLUTE = 2;
static constexpr uint8_t BINARY_FILTER = 3;

/** Default values for base and multiplier */
static constexpr double BASE_DEFAULT = 0.0;
static constexpr double MULTIPLIER_DEFAULT = 1.0;

/** Speed filter constants */
static constexpr int8_t SPEED_MASK_UNKNOWN = -1;
static constexpr int8_t SPEED_MASK_NO_LIMIT = 0;
static constexpr double NO_SPEED_LIMIT = 0.0;

}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__COSTMAP_FILTERS__FILTER_VALUES_HPP_
