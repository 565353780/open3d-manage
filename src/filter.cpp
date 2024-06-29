#include "filter.h"
#include "curvature.h"
#include "knn.h"
#include "normal.h"

const nc::NdArray<float> toFilterWeights(const nc::NdArray<float> &curvatures) {
  nc::NdArray<float> clipped_curvatures(curvatures);

  const float Q1 = nc::percentile<float>(clipped_curvatures, 25.0f)[0];
  const float Q3 = nc::percentile<float>(clipped_curvatures, 75.0f)[0];

  const float IQR = Q3 - Q1;

  const float lower_bound = Q1 - 1.5f * IQR;
  const float upper_bound = Q1 + 1.5f * IQR;

  clipped_curvatures = nc::where<float>(
      clipped_curvatures < lower_bound,
      nc::zeros<float>(clipped_curvatures.shape()), clipped_curvatures);
  clipped_curvatures = nc::where<float>(
      clipped_curvatures > upper_bound,
      upper_bound * nc::ones<float>(clipped_curvatures.shape()),
      clipped_curvatures);

  const float mean = nc::mean<float>(clipped_curvatures)[0];
  const nc::NdArray<float> regular_clipped_curvatures = curvatures - mean;

  nc::NdArray<float> weights = nc::ones<float>(clipped_curvatures.shape());
  weights = nc::where(clipped_curvatures >= mean,
                      nc::exp(-regular_clipped_curvatures *
                              regular_clipped_curvatures / mean / mean),
                      weights);

  return weights;
}

const nc::NdArray<float>
toBilateralFilterPts(const nc::NdArray<float> &points, const float &sigma_d,
                     const float &sigma_n, const int &knn_num,
                     const nc::NdArray<float> &curvature_weights,
                     const bool need_smooth) {
  const nc::NdArray<float> normals = toNormals(points, knn_num, need_smooth);
  const nc::NdArray<int64_t> idxs = toKNNIdxs(points, knn_num);

  nc::NdArray<float> filter_points(points);

  for (int i = 0; i < curvature_weights.size(); ++i) {
    float sum_weight = 0;
    float sum_lambda = 0;

    for (int j = 1; j < knn_num; ++j) {
      const int64_t neighboor_point_idx = idxs[knn_num * i + j];

      const float x_diff = points[3 * neighboor_point_idx] - points[3 * i];
      const float y_diff =
          points[3 * neighboor_point_idx + 1] - points[3 * i + 1];
      const float z_diff =
          points[3 * neighboor_point_idx + 2] - points[3 * i + 2];
      const float distance =
          std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);

      const float normal_dot = normals[3 * i] * x_diff +
                               normals[3 * i + 1] * y_diff +
                               normals[3 * i + 2] * z_diff;

      const float weight =
          std::exp(-(distance * distance) / (2.0 * sigma_d * sigma_d) -
                   (normal_dot * normal_dot) / (2.0 * sigma_n * sigma_n));

      sum_weight += weight;
      sum_lambda += weight * normal_dot;
    }

    if (sum_lambda == 0 || sum_weight == 0) {
      continue;
    }

    const float move_dist = sum_lambda / sum_weight * curvature_weights[i];

    for (int j = 0; j < 3; ++j) {
      filter_points[3 * i + j] += move_dist * normals[3 * i + j];
    }
  }

  return filter_points;
}

const nc::NdArray<float>
toDenoisedPts(const nc::NdArray<float> &points, const float &sigma_d,
              const float &sigma_n, const int &curvature_knn_num,
              const int &filter_knn_num, const bool &need_smooth) {
  const nc::NdArray<float> curvatures =
      toCurvaturesByFit(points, curvature_knn_num);

  const nc::NdArray<float> weights = toFilterWeights(curvatures);

  const nc::NdArray<float> filter_points =
      toBilateralFilterPts(points, sigma_d, sigma_n, filter_knn_num, weights);

  return filter_points;
}
