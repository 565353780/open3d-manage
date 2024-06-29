#include "pcd.h"
#include <Eigen/Core>
#include <memory>

std::shared_ptr<open3d::geometry::PointCloud>
toPcd(const nc::NdArray<float> &points) {
  const int point_num = points.shape().rows;

  std::shared_ptr<open3d::geometry::PointCloud> pcd =
      std::make_shared<open3d::geometry::PointCloud>();
  pcd->points_.reserve(point_num);

  for (int i = 0; i < point_num; ++i) {
    pcd->points_.emplace_back(
        Eigen::Vector3d(points(i, 0), points(i, 1), points(i, 2)));
  }

  return pcd;
}
