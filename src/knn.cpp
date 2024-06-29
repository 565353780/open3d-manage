#include "knn.h"
#include "pcd.h"
#include <memory>
#include <open3d/Open3D.h>
#include <open3d/visualization/utility/DrawGeometry.h>

const nc::NdArray<int64_t> toKNNIdxs(const nc::NdArray<float> &points,
                                     const int &knn_num) {
  const int point_num = points.shape().rows;

  std::shared_ptr<open3d::geometry::PointCloud> pcd = toPcd(points);

  open3d::geometry::KDTreeFlann pcd_tree(*pcd);

  nc::NdArray<int64_t> knn_idxs = nc::zeros<int64_t>(point_num, knn_num);

  for (int i = 0; i < point_num; ++i) {
    std::vector<int> indices;
    std::vector<double> distances;

    pcd_tree.SearchKNN(pcd->points_[i], knn_num, indices, distances);

    for (int j = 0; j < indices.size(); ++j) {
      knn_idxs(i, j) = indices[j];
    }
  }

  return knn_idxs;
}
