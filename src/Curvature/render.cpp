#include "Curvature/render.h"

Eigen::Vector3d scalar_to_color(const double &scalar, const double &min_val,
                                const double &max_val) {
  double value = (scalar - min_val) / (max_val - min_val);
  double r = 1.0, g = 1.0, b = 1.0;

  if (value < 0.5) {
    r = value * 2.0;
    g = value * 2.0;
    b = 1.0;
  } else {
    r = 1.0;
    g = 1.0 - (value - 0.5) * 2.0;
    b = 1.0 - (value - 0.5) * 2.0;
  }

  return Eigen::Vector3d(r, g, b);
}
