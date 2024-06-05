#pragma once

#include "Data/nns_index.h"
#include <torch/extension.h>

enum Metric { L1, L2, Linf };

struct NanoFlannIndexHolderBase {
  virtual ~NanoFlannIndexHolderBase() {}
};

std::unique_ptr<NanoFlannIndexHolderBase> BuildKdTree(size_t num_points,
                                                      const float *const points,
                                                      size_t dimension,
                                                      const Metric metric);

class NanoFlannIndex : public NNSIndex {
public:
  NanoFlannIndex();

  NanoFlannIndex(const torch::Tensor &dataset_points);
  NanoFlannIndex(const torch::Tensor &dataset_points,
                 const torch::Dtype &index_dtype);
  ~NanoFlannIndex();
  NanoFlannIndex(const NanoFlannIndex &) = delete;
  NanoFlannIndex &operator=(const NanoFlannIndex &) = delete;

public:
  bool SetTensorData(const torch::Tensor &dataset_points,
                     const torch::Dtype &index_dtype = torch::kInt64) override;

  bool SetTensorData(const torch::Tensor &dataset_points, double radius,
                     const torch::Dtype &index_dtype = torch::kInt64) override {
    std::cout << "NanoFlannIndex::SetTensorData with radius not implemented."
              << std::endl;

    return false;
  }

  std::pair<torch::Tensor, torch::Tensor>
  SearchKnn(const torch::Tensor &query_points, int knn) const override;

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, const torch::Tensor &radii,
               bool sort = true) const override;

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchRadius(const torch::Tensor &query_points, double radius,
               bool sort = true) const override;

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  SearchHybrid(const torch::Tensor &query_points, double radius,
               int max_knn) const override;

protected:
  std::unique_ptr<NanoFlannIndexHolderBase> holder_;
};
