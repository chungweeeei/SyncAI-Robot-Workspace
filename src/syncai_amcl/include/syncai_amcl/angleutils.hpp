#ifndef SYNCAI_AMCL__ANGLEUTILS_HPP_
#define SYNCAI_AMCL__ANGLEUTILS_HPP_

#include <math.h>

namespace syncai_amcl
{

/*
 * @class angleutils
 * @brief Some utilities for working with angles
 */
class angleutils
{
public:
  /*
   * @brief Normalize angles
   * @brief z Angle to normalize
   * @return normalized angle
   */
  static double normalize(double z);

  /*
   * @brief Find minimum distance between 2 angles
   * @brief a Angle 1
   * @brief b Angle 2
   * @return normalized angle difference
   */
  static double angle_diff(double a, double b);
};

inline double angleutils::normalize(double z)
{
  return atan2(sin(z), cos(z));
}

inline double angleutils::angle_diff(double a, double b)
{
  a = normalize(a);
  b = normalize(b);
  double d1 = a - b;
  double d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) {
    d2 *= -1.0;
  }
  if (fabs(d1) < fabs(d2)) {
    return d1;
  } else {
    return d2;
  }
}

}  // namespace syncai_amcl

#endif  // SYNCAI_AMCL__ANGLEUTILS_HPP_
