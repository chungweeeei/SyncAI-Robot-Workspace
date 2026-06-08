#include "syncai_costmap_2d/costmap_math.hpp"

#include <vector>

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
  double A = pX - x0;
  double B = pY - y0;
  double C = x1 - x0;
  double D = y1 - y0;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = dot / len_sq;

  double xx, yy;

  if (param < 0) {
    xx = x0;
    yy = y0;
  } else if (param > 1) {
    xx = x1;
    yy = y1;
  } else {
    xx = x0 + param * C;
    yy = y0 + param * D;
  }

  return distance(pX, pY, xx, yy);
};