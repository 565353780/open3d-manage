#pragma once

#include <Eigen/Core>

// Function to convert scalar values into a colormap (Jet colormap in this case)
Eigen::Vector3d scalar_to_color(const double &scalar, const double &min_val,
                                const double &max_val);
