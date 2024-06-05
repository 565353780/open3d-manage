#include "Data/knn_index.h"

KnnIndex::KnnIndex() {}

KnnIndex::KnnIndex(const torch::Tensor &dataset_points) {
  SetTensorData(dataset_points);
}

KnnIndex::KnnIndex(const torch::Tensor &dataset_points,
                   const torch::Dtype &index_dtype) {
  SetTensorData(dataset_points, index_dtype);
}

KnnIndex::~KnnIndex() {}

bool KnnIndex::SetTensorData(const torch::Tensor &dataset_points,
                             const torch::Dtype &index_dtype) {
  int64_t num_dataset_points = dataset_points.size(0);
  const torch::TensorOptions opts = torch::TensorOptions().dtype(torch::kInt64);
  torch::Tensor points_row_splits = torch::zeros({0, num_dataset_points}, opts);
  return SetTensorData(dataset_points, points_row_splits, index_dtype);
}

bool KnnIndex::SetTensorData(const torch::Tensor &dataset_points,
                             const torch::Tensor &points_row_splits,
                             const torch::Dtype &index_dtype) {
  if (dataset_points.sizes().size() != 2) {
    std::cout << "dataset_points must be 2D matrix with shape "
                 "{n_dataset_points, d}."
              << std::endl;
  }
  if (dataset_points.size(0) <= 0 || dataset_points.size(1) <= 0) {
    std::cout << "Failed due to no data." << std::endl;
  }
  if (dataset_points.size(0) != points_row_splits[-1].item<int64_t>()) {
    std::cout << "dataset_points and points_row_splits have incompatible "
                 "shapes."
              << std::endl;
  }

  return false;
}

std::pair<torch::Tensor, torch::Tensor>
KnnIndex::SearchKnn(const torch::Tensor &query_points, int knn) const {
  int64_t num_query_points = query_points.size(0);
  const torch::TensorOptions opts = torch::TensorOptions().dtype(torch::kInt64);
  torch::Tensor queries_row_splits = torch::zeros({0, num_query_points}, opts);
  return SearchKnn(query_points, queries_row_splits, knn);
}

std::pair<torch::Tensor, torch::Tensor>
KnnIndex::SearchKnn(const torch::Tensor &query_points,
                    const torch::Tensor &queries_row_splits, int knn) const {
  const torch::Dtype dtype = GetDtype();
  const torch::Device device = GetDevice();

  if (query_points.size(0) != queries_row_splits[-1].item<int64_t>()) {
    std::cout << "query_points and queries_row_splits have incompatible "
                 "shapes."
              << std::endl;
  }
  if (knn <= 0) {
    std::cout << "knn should be larger than 0." << std::endl;
  }

  torch::Tensor query_points_ = query_points.contiguous();
  torch::Tensor queries_row_splits_ = queries_row_splits.contiguous();

  torch::Tensor neighbors_index, neighbors_distance;
  const torch::TensorOptions opts = torch::TensorOptions().dtype(torch::kInt64);
  torch::Tensor neighbors_row_splits =
      torch::empty({query_points.size(0) + 1}, opts);

  return std::make_pair(neighbors_index, neighbors_distance);
}
