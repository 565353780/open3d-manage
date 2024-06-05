#pragma once

#include <torch/extension.h>

class NNSIndex {
public:
  NNSIndex() {}
  virtual ~NNSIndex() {}
  NNSIndex(const NNSIndex &) = delete;
  NNSIndex &operator=(const NNSIndex &) = delete;

public:
  virtual bool SetTensorData(const torch::Tensor &dataset_points,
                             const torch::Dtype &index_dtype) = 0;

  virtual bool SetTensorData(const torch::Tensor &dataset_points, double radius,
                             const torch::Dtype &index_dtype) = 0;

  virtual std::pair<torch::Tensor, torch::Tensor>
  SearchKnn(const torch::Tensor &query_points, int knn) const = 0;

  virtual std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, const torch::Tensor &radii,
               bool sort) const = 0;

  virtual std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, double radius,
               bool sort) const = 0;

  virtual std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchHybrid(const torch::Tensor &query_points, double radius,
               int max_knn) const = 0;

  int GetDimension() const;

  size_t GetDatasetSize() const;

  torch::Dtype GetDtype() const;

  torch::Device GetDevice() const;

  torch::Dtype GetIndexDtype() const;

protected:
  torch::Tensor dataset_points_;
  torch::Dtype index_dtype_;
};
