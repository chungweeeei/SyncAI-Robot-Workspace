#ifndef SYNCAI_NAV2_COSTMAP_2D__COST_VALUES_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__COST_VALUES_HPP_

namespace syncai_costmap_2d
{

enum class CombinationMethod : int {
  /**
    * The CombinationMethod enum class defines the policy for how each layer in the costmap
    * merges its own values into the master costmap.
    * Overwrite - Overwrite the master's value directly (but NO_INFORMATION is not written)
    *           - Static layer: the static map is ground truth, so it overwrites outright
    * Max - Take the larger of master and layer. NO_INFORMATION also gets overwritten by the
    *       larger valid value
    *     - ObstacleLayer: a detected obstacle must be kept (lethal is the largest value)
    * MaxWithoutUnknownOverwrite - Same as Max, but keeps the master's original NO_INFORMATION
    *                              (does not let unknown be overwritten as known)
    *                            - For cases that want to keep the "unknown stays unknown"
    *                              semantics, e.g. certain inflation usages
    * Max is the costmap's default combination method.
    */

  Overwrite = 0,
  Max = 1,
  MaxWithoutUnknownOverwrite = 2
};

static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_NAV2_COSTMAP_2D__COST_VALUES_HPP_