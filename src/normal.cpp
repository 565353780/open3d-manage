#include "normal.h"
#include "pcd.h"
#include <Eigen/Core>
#include <open3d/Open3D.h>

const nc::NdArray<float> toNormals(const nc::NdArray<float> &points,
                                   const int &knn_num,
                                   const bool &need_smooth) {
  const int point_num = points.shape().rows;

  open3d::geometry::PointCloud pcd = toPcd(points);

  if (knn_num > 0) {
    pcd.EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(knn_num));
  } else {
    pcd.EstimateNormals();
  }

  pcd.NormalizeNormals();

  if (need_smooth > 0) {
    pcd.OrientNormalsConsistentTangentPlane(knn_num);
  }

  nc::NdArray<float> normals = nc::zeros<float>(point_num, 3);

  for (int i = 0; i < point_num; ++i) {
    normals(i, 0) = pcd.normals_[i][0];
    normals(i, 1) = pcd.normals_[i][1];
    normals(i, 2) = pcd.normals_[i][2];
  }

  return normals;
}
