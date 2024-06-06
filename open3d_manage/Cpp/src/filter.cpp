#include "filter.h"

using namespace pybind11::literals;

Filter::Filter(const std::string root_path) {
  py::gil_scoped_acquire acquire;

  py::object sys = py::module_::import("sys");

  sys.attr("path").attr("append")(root_path);

  normal_ = py::module_::import("open3d_manage.Method.normal");

  knn_ = py::module_::import("open3d_manage.Method.knn");

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

const std::vector<float> Filter::toDenoisedPts(const std::vector<float> &points,
                                               const int &knn_num,
                                               const bool &need_smooth) {
  return;
}
