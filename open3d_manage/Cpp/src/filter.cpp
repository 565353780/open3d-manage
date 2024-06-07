#include "filter.h"
#include <iostream>

using namespace pybind11::literals;

Filter::Filter(const std::string root_path) {
  py::gil_scoped_acquire acquire;

  py::object sys = py::module_::import("sys");

  sys.attr("path").attr("append")(root_path);

  normal_ = py::module_::import("open3d_manage.Method.normal");

  knn_ = py::module_::import("open3d_manage.Method.knn");

  curvature_ = py::module_::import("open3d_manage.Method.curvature");

  filter_ = py::module_::import("open3d_manage.Method.filter");

  return;
}

Filter::~Filter() {}

const std::vector<float> Filter::toNormals(const std::vector<float> &points,
                                           const int &knn_num,
                                           const bool &need_smooth) {
  py::gil_scoped_acquire acquire;

  py::list point_list;
  for (int i = 0; i < points.size(); ++i) {
    point_list.append(points[i]);
  }

  py::list normal_list = normal_.attr("toNormalsList")(
      "points"_a = point_list, "knn_num"_a = knn_num,
      "need_smooth"_a = need_smooth);

  std::vector<float> normals;
  normals.reserve(points.size());

  for (int i = 0; i < normal_list.size(); ++i) {
    normals.emplace_back(normal_list[i].cast<float>());
  }

  return normals;
}

const std::vector<int64_t> Filter::toKNNIdxs(const std::vector<float> &points,
                                             const int &knn_num) {
  py::gil_scoped_acquire acquire;

  py::list point_list;
  for (int i = 0; i < points.size(); ++i) {
    point_list.append(points[i]);
  }

  py::list knn_idxs_list = knn_.attr("toKNNIdxsList")("points"_a = point_list,
                                                      "knn_num"_a = knn_num);

  std::vector<int64_t> knn_idxs;
  knn_idxs.reserve(points.size());

  for (int i = 0; i < knn_idxs_list.size(); ++i) {
    knn_idxs.emplace_back(knn_idxs_list[i].cast<int64_t>());
  }

  return knn_idxs;
}

const std::vector<float>
Filter::toCurvaturesByFit(const std::vector<float> points, const int &knn_num) {
  py::gil_scoped_acquire acquire;

  py::list point_list;
  for (int i = 0; i < points.size(); ++i) {
    point_list.append(points[i]);
  }

  py::list curvatures_list = curvature_.attr("toCurvaturesListByFit")(
      "points"_a = point_list, "knn_num"_a = knn_num);

  std::vector<float> curvatures;
  curvatures.reserve(curvatures_list.size());

  for (int i = 0; i < curvatures_list.size(); ++i) {
    curvatures.emplace_back(curvatures_list[i].cast<float>());
  }

  return curvatures;
}

const std::vector<float>
Filter::toFilterWeights(const std::vector<float> &curvatures) {
  py::gil_scoped_acquire acquire;

  py::list curvature_list;
  for (int i = 0; i < curvatures.size(); ++i) {
    curvature_list.append(curvatures[i]);
  }

  py::list weights_list =
      filter_.attr("toFilterWeightsList")("curvatures"_a = curvature_list);

  std::vector<float> weights;
  weights.reserve(weights_list.size());

  for (int i = 0; i < weights_list.size(); ++i) {
    weights.emplace_back(weights_list[i].cast<float>());
  }

  return weights;
}

const std::vector<float> Filter::toBilateralFilterPts(
    const std::vector<float> &points, const float &sigma_d,
    const float &sigma_n, const int &knn_num,
    const std::vector<float> &curvature_weights, const bool need_smooth) {
  const std::vector<float> normals = toNormals(points, knn_num, need_smooth);
  const std::vector<int64_t> idxs = toKNNIdxs(points, knn_num);

  std::vector<float> filter_points(points);

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

    std::cout << move_dist << std::endl;

    for (int j = 0; j < 3; ++j) {
      filter_points[3 * i + j] += move_dist * normals[3 * i + j];
    }
  }

  return filter_points;
}

const std::vector<float>
Filter::toDenoisedPts(const std::vector<float> &points, const float &sigma_d,
                      const float &sigma_n, const int &curvature_knn_num,
                      const int &filter_knn_num, const bool &need_smooth) {
  const std::vector<float> curvatures =
      toCurvaturesByFit(points, curvature_knn_num);

  const std::vector<float> weights = toFilterWeights(curvatures);

  const std::vector<float> filter_points =
      toBilateralFilterPts(points, sigma_d, sigma_n, filter_knn_num, weights);

  return filter_points;
}
