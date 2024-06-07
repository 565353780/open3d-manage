#pragma once

#include <pybind11/embed.h>
#include <string>

namespace py = pybind11;

class __attribute__((visibility("default"))) Filter {
public:
  Filter(const std::string root_path = "../../open3d-manage/");
  ~Filter();

  const std::vector<float> toNormals(const std::vector<float> &points,
                                     const int &knn_num = 0,
                                     const bool &need_smooth = false);

  const std::vector<int64_t> toKNNIdxs(const std::vector<float> &points,
                                       const int &knn_num);

  const std::vector<float> toCurvaturesByFit(const std::vector<float> points,
                                             const int &knn_num);

  const std::vector<float>
  toFilterWeights(const std::vector<float> &curvatures);

  const std::vector<float>
  toBilateralFilterPts(const std::vector<float> &points, const float &sigma_d,
                       const float &sigma_n, const int &knn_num,
                       const std::vector<float> &curvature_weights,
                       const bool need_smooth = false);

  const std::vector<float>
  toDenoisedPts(const std::vector<float> &points, const float &sigma_d,
                const float &sigma_n, const int &curvature_knn_num,
                const int &filter_knn_num, const bool &need_smooth = false);

private:
  py::scoped_interpreter guard_{};

  py::object normal_;
  py::object knn_;
  py::object curvature_;
  py::object filter_;
};
