#include "filter.h"
#include <iostream>

int main() {
  // super params
  const float sigma_d = 200;
  const float sigma_n = 2000;
  const int curvature_knn_num = 10;
  const int filter_knn_num = 40;
  const bool need_smooth = true;

  // input point cloud [x1, y1, z1, x2, y2, z2, ...]
  std::vector<float> points;
  points.resize(3000);
  for (int i = 0; i < 1000; ++i) {
    points[3 * i] = 1.0 * i;
    points[3 * i + 1] = 2.0 * i;
    points[3 * i + 2] = 3.0 * i;
  }

  // filter method call
  const std::vector<float> denoised_points = toDenoisedPts(
      points, sigma_d, sigma_n, curvature_knn_num, filter_knn_num, need_smooth);

  // result output demo
  std::cout << "input points size: " << points.size() / 3 << std::endl;
  std::cout << "denoised points size: " << denoised_points.size() / 3
            << std::endl;

  std::cout << "from: [" << points[0] << "," << points[1] << "," << points[2]
            << "]" << std::endl;
  std::cout << "to: [" << denoised_points[0] << "," << denoised_points[1] << ","
            << denoised_points[2] << "]" << std::endl;

  return 1;
}
