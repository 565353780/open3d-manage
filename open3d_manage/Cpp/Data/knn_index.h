#include "Data/nns_index.h"
#include <torch/extension.h>

class KnnIndex : public NNSIndex {
public:
  KnnIndex();

  KnnIndex(const torch::Tensor &dataset_points);
  KnnIndex(const torch::Tensor &dataset_points,
           const torch::Dtype &index_dtype);
  ~KnnIndex();
  KnnIndex(const KnnIndex &) = delete;
  KnnIndex &operator=(const KnnIndex &) = delete;

public:
  bool SetTensorData(const torch::Tensor &dataset_points,
                     const torch::Dtype &index_dtype = torch::kInt64) override;
  bool SetTensorData(const torch::Tensor &dataset_points,
                     const torch::Tensor &points_row_splits,
                     const torch::Dtype &index_dtype = torch::kInt64);
  bool SetTensorData(const torch::Tensor &dataset_points, double radius,
                     const torch::Dtype &index_dtype = torch::kInt64) override {
    std::cout << "[KnnIndex::SetTensorData with radius not implemented."
              << std::endl;

    return false;
  }

  std::pair<torch::Tensor, torch::Tensor>
  SearchKnn(const torch::Tensor &query_points, int knn) const override;

  std::pair<torch::Tensor, torch::Tensor>
  SearchKnn(const torch::Tensor &query_points,
            const torch::Tensor &queries_row_splits, int knn) const;

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, const torch::Tensor &radii,
               bool sort) const override {
    std::cout << "KnnIndex::SearchRadius not implemented." << std::endl;

    return std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>();
  }

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, const double radius,
               bool sort) const override {
    std::cout << "KnnIndex::SearchRadius not implemented." << std::endl;

    return std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>();
  }

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchHybrid(const torch::Tensor &query_points, const double radius,
               const int max_knn) const override {
    std::cout << "KnnIndex::SearchHybrid not implemented." << std::endl;

    return std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>();
  }

protected:
  torch::Tensor points_row_splits_;
};
