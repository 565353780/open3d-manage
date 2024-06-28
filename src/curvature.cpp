#include "curvature.h"
#include "knn.h"

const nc::NdArray<float> toCurvaturesByFit(const nc::NdArray<float> &points,
                                           const int &knn_num) {
  const int point_num = points.shape().rows;

  const nc::NdArray<int64_t> idxs = toKNNIdxs(points, knn_num);

  nc::NdArray<float> curvatures = nc::zeros<float>(point_num);

  for (int i = 0; i < point_num; ++i) {
    for (int j = 0; j < knn_num; ++j) {
      const int64_t idx = idxs[knn_num * i + j];
    }
  }

  return curvatures;
}
