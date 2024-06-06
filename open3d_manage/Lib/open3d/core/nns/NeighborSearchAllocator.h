// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include <torch/extension.h>

template <class T, class TIndex = int32_t> class NeighborSearchAllocator {
public:
  NeighborSearchAllocator(torch::Device device) : device_(device) {}

  void AllocIndices(TIndex **ptr, size_t num) {
    indices_ = torch::empty(
        {int64_t(num)},
        torch::TensorOptions().dtype(torch::kInt32).device(device_));
    *ptr = indices_.data_ptr<TIndex>();
  }

  void AllocIndices(TIndex **ptr, size_t num, TIndex value) {
    indices_ =
        torch::ones(
            {int64_t(num)},
            torch::TensorOptions().dtype(torch::kInt32).device(device_)) *
        value;
    *ptr = indices_.data_ptr<TIndex>();
  }

  void AllocDistances(T **ptr, size_t num) {
    distances_ = torch::empty(
        {int64_t(num)},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    *ptr = distances_.data_ptr<T>();
  }

  void AllocDistances(T **ptr, size_t num, T value) {
    distances_ =
        torch::ones(
            {int64_t(num)},
            torch::TensorOptions().dtype(torch::kFloat32).device(device_)) *
        value;
    *ptr = distances_.data_ptr<T>();
  }

  void AllocCounts(TIndex **ptr, size_t num) {
    counts_ = torch::empty(
        {int64_t(num)},
        torch::TensorOptions().dtype(torch::kInt32).device(device_));
    *ptr = counts_.data_ptr<TIndex>();
  }

  void AllocCounts(TIndex **ptr, size_t num, TIndex value) {
    counts_ = torch::ones(
                  {int64_t(num)},
                  torch::TensorOptions().dtype(torch::kInt32).device(device_)) *
              value;
    *ptr = counts_.data_ptr<TIndex>();
  }

  const TIndex *IndicesPtr() const { return indices_.data_ptr<TIndex>(); }

  const T *DistancesPtr() const { return distances_.data_ptr<T>(); }

  const TIndex *CountsPtr() const { return counts_.data_ptr<TIndex>(); }

  const torch::Tensor &NeighborsIndex() const { return indices_; }
  torch::Tensor &NeighborsIndex_() { return indices_; }
  const torch::Tensor &NeighborsDistance() const { return distances_; }
  torch::Tensor &NeighborsDistance_() { return distances_; }
  const torch::Tensor &NeighborsCount() const { return counts_; }

private:
  torch::Tensor indices_;
  torch::Tensor distances_;
  torch::Tensor counts_;
  torch::Device device_;
};
