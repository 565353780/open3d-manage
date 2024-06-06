#pragma once

#include <pybind11/embed.h>
#include <string>

namespace py = pybind11;

class __attribute__((visibility("default"))) Filter {
public:
  Filter(const std::string root_path = "../../open3d-manage/");
  ~Filter();

  const std::vector<float> toNormals(const std::vector<float> &points,
                                     const int &knn_num,
                                     const bool &need_smooth);

  const std::vector<int64_t> toKNNIdxs(const std::vector<float> &points,
                                       const int &knn_num);

  const std::vector<float> toDenoisedPts(const std::vector<float> &points,
                                         const int &knn_num,
                                         const bool &need_smooth);

private:
  py::scoped_interpreter guard_{};

  py::object normal_;
  py::object knn_;
};
