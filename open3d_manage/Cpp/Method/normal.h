#pragma once

#include <torch/extension.h>

void EstimatePointWiseRobustNormalizedCovarianceKernel(
    const float *points_ptr, const int32_t *indices_ptr,
    const int32_t &indices_count, float *covariance_ptr);

void EstimateCovariancesUsingHybridSearchCPU(const torch::Tensor &points,
                                             torch::Tensor &covariances,
                                             const double &radius,
                                             const int64_t &max_nn);

void EstimateCovariancesUsingRadiusSearchCPU(const torch::Tensor &points,
                                             torch::Tensor &covariances,
                                             const double &radius);

void EstimateCovariancesUsingKNNSearchCPU(const torch::Tensor &points,
                                          torch::Tensor &covariances,
                                          const int64_t &max_nn);

void EstimatePointWiseNormalsWithFastEigen3x3(const float *covariance_ptr,
                                              float *normals_ptr);

void EstimateNormalsFromCovariancesCPU(const torch::Tensor &covariances,
                                       torch::Tensor &normals,
                                       const bool has_normals);

void EstimateNormals(const torch::Tensor &points, const int max_knn = 30,
                     const double radius = -1.0);
