#include "Module/nns.h"

NearestNeighborSearch::~NearestNeighborSearch(){};

bool NearestNeighborSearch::SetIndex() {
  nanoflann_index_.reset(new NanoFlannIndex());
  return nanoflann_index_->SetTensorData(dataset_points_, index_dtype_);
};

bool NearestNeighborSearch::KnnIndex() { return SetIndex(); };

bool NearestNeighborSearch::MultiRadiusIndex() { return SetIndex(); };

bool NearestNeighborSearch::FixedRadiusIndex(double radius) {
  return SetIndex();
}

bool NearestNeighborSearch::HybridIndex(double radius) { return SetIndex(); };

std::pair<torch::Tensor, torch::Tensor>
NearestNeighborSearch::KnnSearch(const torch::Tensor &query_points, int knn) {
  return nanoflann_index_->SearchKnn(query_points, knn);
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
NearestNeighborSearch::FixedRadiusSearch(const torch::Tensor &query_points,
                                         double radius, bool sort) {
  return nanoflann_index_->SearchRadius(query_points, radius);
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
NearestNeighborSearch::MultiRadiusSearch(const torch::Tensor &query_points,
                                         const torch::Tensor &radii) {
  return nanoflann_index_->SearchRadius(query_points, radii);
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
NearestNeighborSearch::HybridSearch(const torch::Tensor &query_points,
                                    const double radius,
                                    const int max_knn) const {
  return nanoflann_index_->SearchHybrid(query_points, radius, max_knn);
}
