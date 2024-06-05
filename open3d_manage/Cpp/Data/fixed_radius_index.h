#pragma once

#include "Data/nns_index.h"
#include <torch/extension.h>

class FixedRadiusIndex : public NNSIndex {
public:
  FixedRadiusIndex();

  FixedRadiusIndex(const torch::Tensor &dataset_points, double radius);
  FixedRadiusIndex(const torch::Tensor &dataset_points, double radius,
                   const torch::Dtype &index_dtype);
  ~FixedRadiusIndex();
  FixedRadiusIndex(const FixedRadiusIndex &) = delete;
  FixedRadiusIndex &operator=(const FixedRadiusIndex &) = delete;

public:
  bool SetTensorData(const torch::Tensor &dataset_points,
                     const torch::Dtype &index_dtype = torch::kInt64) override {
    std::cout << "FixedRadiusIndex::SetTensorData without radius not "
                 "implemented."
              << std::endl;

    return false;
  }

  bool SetTensorData(const torch::Tensor &dataset_points, double radius,
                     const torch::Dtype &index_dtype = torch::kInt64) override;
  bool SetTensorData(const torch::Tensor &dataset_points,
                     const torch::Tensor &points_row_splits, double radius,
                     const torch::Dtype &index_dtype = torch::kInt64);

  std::pair<torch::Tensor, torch::Tensor>
  SearchKnn(const torch::Tensor &query_points, int knn) const override {
    std::cout << "FixedRadiusIndex::SearchKnn not implemented." << std::endl;

    return std::pair<torch::Tensor, torch::Tensor>();
  }

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, const torch::Tensor &radii,
               bool sort = true) const override {
    std::cout << "FixedRadiusIndex::SearchRadius with multi-radii not "
                 "implemented."
              << std::endl;

    return std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>();
  }

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, double radius,
               bool sort = true) const override;
  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points,
               const torch::Tensor &queries_row_splits, double radius,
               bool sort = true) const;

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchHybrid(const torch::Tensor &query_points, double radius,
               int max_knn) const override;

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchHybrid(const torch::Tensor &query_points,
               const torch::Tensor &queries_row_splits, double radius,
               int max_knn) const;

  const double hash_table_size_factor = 1.0 / 32;
  const int64_t max_hash_tabls_size = 33554432;

protected:
  torch::Tensor points_row_splits_;
  torch::Tensor hash_table_splits_;
  torch::Tensor hash_table_cell_splits_;
  torch::Tensor hash_table_index_;
};
