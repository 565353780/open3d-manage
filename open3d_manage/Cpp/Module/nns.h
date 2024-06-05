#pragma once

#include "Data/knn_index.h"
#include "Data/nano_flann_index.h"
#include <torch/extension.h>

class NearestNeighborSearch {
public:
  NearestNeighborSearch(const torch::Tensor &dataset_points,
                        const torch::Dtype &index_dtype = torch::kInt32)
      : dataset_points_(dataset_points), index_dtype_(index_dtype){};
  ~NearestNeighborSearch();
  NearestNeighborSearch(const NearestNeighborSearch &) = delete;
  NearestNeighborSearch &operator=(const NearestNeighborSearch &) = delete;

public:
  bool KnnIndex();

  bool MultiRadiusIndex();

  bool FixedRadiusIndex(double radius = -1.0);

  bool HybridIndex(double radius = -1.0);

  std::pair<torch::Tensor, torch::Tensor>
  KnnSearch(const torch::Tensor &query_points, int knn);

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  FixedRadiusSearch(const torch::Tensor &query_points, double radius,
                    bool sort = true);

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  MultiRadiusSearch(const torch::Tensor &query_points,
                    const torch::Tensor &radii);

  std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
  HybridSearch(const torch::Tensor &query_points, const double radius,
               const int max_knn) const;

private:
  bool SetIndex();

protected:
  std::unique_ptr<NanoFlannIndex> nanoflann_index_;
  // std::unique_ptr<KnnIndex> knn_index_;
  const torch::Tensor dataset_points_;
  const torch::Dtype index_dtype_;
};
