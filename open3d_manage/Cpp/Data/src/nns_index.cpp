#include "Data/nns_index.h"

int NNSIndex::GetDimension() const { return dataset_points_.size(1); }

size_t NNSIndex::GetDatasetSize() const { return dataset_points_.size(0); }

torch::Dtype NNSIndex::GetDtype() const {
  return dataset_points_.scalar_type();
}

torch::Device NNSIndex::GetDevice() const { return dataset_points_.device(); }

torch::Dtype NNSIndex::GetIndexDtype() const { return index_dtype_; }
