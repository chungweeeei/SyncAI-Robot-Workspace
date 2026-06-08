#ifndef SYNCAI_NAV2_COSTMAP_2D__COSTMAP_MATH_HPP_
#define SYNCAI_NAV2_COSTMAP_2D__COSTMAP_MATH_HPP_

#include <math.h>

#include <algorithm>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

/** @brief Return -1 if x < 0, +1 otherwise. */
inline double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

/** @brief Same as sign(x) but returns 0 if x is 0. */
inline double sign0(double x)
{
  return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
}

/** @brief Gets L2 norm distance */
inline double distance(double x0, double y0, double x1, double y1)
{
  return hypot(x1 - x0, y1 - y0);
}

/** @brief Gets point distance to a line */
double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

#endif  // SYNCAI_NAV2_COSTMAP_2D__COSTMAP_MATH_HPP_
