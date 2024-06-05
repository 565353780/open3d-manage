#include "Method/normal.h"
#include "Module/nns.h"
#include <ATen/core/ATen_fwd.h>

void EstimatePointWiseRobustNormalizedCovarianceKernel(
    const float *points_ptr, const int32_t *indices_ptr,
    const int32_t &indices_count, float *covariance_ptr) {
  if (indices_count < 3) {
    covariance_ptr[0] = 1.0;
    covariance_ptr[1] = 0.0;
    covariance_ptr[2] = 0.0;
    covariance_ptr[3] = 0.0;
    covariance_ptr[4] = 1.0;
    covariance_ptr[5] = 0.0;
    covariance_ptr[6] = 0.0;
    covariance_ptr[7] = 0.0;
    covariance_ptr[8] = 1.0;
    return;
  }

  double centroid[3] = {0};
  for (int32_t i = 0; i < indices_count; ++i) {
    int32_t idx = 3 * indices_ptr[i];
    centroid[0] += points_ptr[idx];
    centroid[1] += points_ptr[idx + 1];
    centroid[2] += points_ptr[idx + 2];
  }

  centroid[0] /= indices_count;
  centroid[1] /= indices_count;
  centroid[2] /= indices_count;

  // cumulants must always be Float64 to ensure precision.
  double cumulants[6] = {0};
  for (int32_t i = 0; i < indices_count; ++i) {
    int32_t idx = 3 * indices_ptr[i];
    const double x = static_cast<double>(points_ptr[idx]) - centroid[0];
    const double y = static_cast<double>(points_ptr[idx + 1]) - centroid[1];
    const double z = static_cast<double>(points_ptr[idx + 2]) - centroid[2];

    cumulants[0] += x * x;
    cumulants[1] += y * y;
    cumulants[2] += z * z;

    cumulants[3] += x * y;
    cumulants[4] += x * z;
    cumulants[5] += y * z;
  }

  // Using Bessel's correction (dividing by (n - 1) instead of n).
  // Refer:
  // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
  const double normalization_factor = static_cast<double>(indices_count - 1);
  for (int i = 0; i < 6; ++i) {
    cumulants[i] /= normalization_factor;
  }

  // Covariances(0, 0)
  covariance_ptr[0] = static_cast<float>(cumulants[0]);
  // Covariances(1, 1)
  covariance_ptr[4] = static_cast<float>(cumulants[1]);
  // Covariances(2, 2)
  covariance_ptr[8] = static_cast<float>(cumulants[2]);

  // Covariances(0, 1) = Covariances(1, 0)
  covariance_ptr[1] = static_cast<float>(cumulants[3]);
  covariance_ptr[3] = covariance_ptr[1];

  // Covariances(0, 2) = Covariances(2, 0)
  covariance_ptr[2] = static_cast<float>(cumulants[4]);
  covariance_ptr[6] = covariance_ptr[2];

  // Covariances(1, 2) = Covariances(2, 1)
  covariance_ptr[5] = static_cast<float>(cumulants[5]);
  covariance_ptr[7] = covariance_ptr[5];
}

void EstimateCovariancesUsingHybridSearchCPU(const torch::Tensor &points,
                                             torch::Tensor &covariances,
                                             const double &radius,
                                             const int64_t &max_nn) {
  int64_t n = points.size(0);

  NearestNeighborSearch tree(points, torch::kInt32);
  bool check = tree.HybridIndex(radius);
  if (!check) {
    std::cout << "Building FixedRadiusIndex failed." << std::endl;

    return;
  }

  torch::Tensor indices, distance, counts;
  std::tie(indices, distance, counts) =
      tree.HybridSearch(points, radius, max_nn);

  const float *points_ptr = points.data_ptr<float>();
  int32_t *neighbour_indices_ptr = indices.data_ptr<int32_t>();
  int32_t *neighbour_counts_ptr = counts.data_ptr<int32_t>();
  float *covariances_ptr = covariances.data_ptr<float>();

  for (int i = 0; i < indices.size(0); ++i) {
    const int32_t neighbour_offset = max_nn * i;
    const int32_t neighbour_count = neighbour_counts_ptr[i];
    const int32_t covariances_offset = 9 * i;

    EstimatePointWiseRobustNormalizedCovarianceKernel(
        points_ptr, neighbour_indices_ptr + neighbour_offset, neighbour_count,
        covariances_ptr + covariances_offset);
  }

  return;
}

void EstimateCovariancesUsingRadiusSearchCPU(const torch::Tensor &points,
                                             torch::Tensor &covariances,
                                             const double &radius) {
  int64_t n = points.size(0);

  NearestNeighborSearch tree(points, torch::kInt32);
  bool check = tree.FixedRadiusIndex(radius);
  if (!check) {
    std::cout << "Building Radius-Index failed." << std::endl;

    return;
  }

  torch::Tensor indices, distance, counts;
  std::tie(indices, distance, counts) = tree.FixedRadiusSearch(points, radius);

  const float *points_ptr = points.data_ptr<float>();
  const int32_t *neighbour_indices_ptr = indices.data_ptr<int32_t>();
  const int32_t *neighbour_counts_ptr = counts.data_ptr<int32_t>();
  float *covariances_ptr = covariances.data_ptr<float>();

  for (int i = 0; i < indices.size(0); ++i) {
    const int32_t neighbour_offset = neighbour_counts_ptr[i];
    const int32_t neighbour_count =
        (neighbour_counts_ptr[i + 1] - neighbour_counts_ptr[i]);
    const int32_t covariances_offset = 9 * i;

    EstimatePointWiseRobustNormalizedCovarianceKernel(
        points_ptr, neighbour_indices_ptr + neighbour_offset, neighbour_count,
        covariances_ptr + covariances_offset);
  }

  return;
}
void EstimateCovariancesUsingKNNSearchCPU(const torch::Tensor &points,
                                          torch::Tensor &covariances,
                                          const int64_t &max_nn) {
  int64_t n = points.size(0);

  NearestNeighborSearch tree(points, torch::kInt32);
  bool check = tree.KnnIndex();
  if (!check) {
    std::cout << "Building KNN-Index failed." << std::endl;

    return;
  }

  torch::Tensor indices, distance;
  std::tie(indices, distance) = tree.KnnSearch(points, max_nn);

  indices = indices.contiguous();
  int32_t nn_count = static_cast<int32_t>(indices.size(1));

  if (nn_count < 3) {
    std::cout << "Not enough neighbors to compute Covariances / Normals. "
                 "Try "
                 "increasing the max_nn parameter."
              << std::endl;

    return;
  }

  auto points_ptr = points.data_ptr<float>();
  auto neighbour_indices_ptr = indices.data_ptr<int32_t>();
  auto covariances_ptr = covariances.data_ptr<float>();

  for (int i = 0; i < indices.size(0); ++i) {
    const int32_t neighbour_offset = nn_count * i;

    const int32_t covariances_offset = 9 * i;

    EstimatePointWiseRobustNormalizedCovarianceKernel(
        points_ptr, neighbour_indices_ptr + neighbour_offset, nn_count,
        covariances_ptr + covariances_offset);
  }
}

void EstimatePointWiseNormalsWithFastEigen3x3(const float *covariance_ptr,
                                              float *normals_ptr) {
  float max_coeff = covariance_ptr[0];

  for (int i = 1; i < 9; ++i) {
    if (max_coeff < covariance_ptr[i]) {
      max_coeff = covariance_ptr[i];
    }
  }

  if (max_coeff == 0) {
    normals_ptr[0] = 0.0;
    normals_ptr[1] = 0.0;
    normals_ptr[2] = 0.0;
    return;
  }

  float A[9] = {0};

  for (int i = 0; i < 9; ++i) {
    A[i] = covariance_ptr[i] / max_coeff;
  }

  float norm = A[1] * A[1] + A[2] * A[2] + A[5] * A[5];

  if (norm > 0) {
    float eval[3];
    float evec0[3];
    float evec1[3];
    float evec2[3];

    float q = (A[0] + A[4] + A[8]) / 3.0;

    float b00 = A[0] - q;
    float b11 = A[4] - q;
    float b22 = A[8] - q;

    float p = sqrt((b00 * b00 + b11 * b11 + b22 * b22 + norm * 2.0) / 6.0);

    float c00 = b11 * b22 - A[5] * A[5];
    float c01 = A[1] * b22 - A[5] * A[2];
    float c02 = A[1] * A[5] - b11 * A[2];
    float det = (b00 * c00 - A[1] * c01 + A[2] * c02) / (p * p * p);

    float half_det = det * 0.5;
    half_det =
        min(max(half_det, static_cast<float>(-1.0)), static_cast<float>(1.0));

    float angle = acos(half_det) / 3.0;
    const float two_thrids_pi = 2.09439510239319549;

    float beta2 = cos(angle) * 2.0;
    float beta0 = cos(angle + two_thrids_pi) * 2.0;
    float beta1 = -(beta0 + beta2);

    eval[0] = q + p * beta0;
    eval[1] = q + p * beta1;
    eval[2] = q + p * beta2;

    if (half_det >= 0) {
      ComputeEigenvector0<float>(A, eval[2], evec2);

      if (eval[2] < eval[0] && eval[2] < eval[1]) {
        normals_ptr[0] = evec2[0];
        normals_ptr[1] = evec2[1];
        normals_ptr[2] = evec2[2];

        return;
      }

      ComputeEigenvector1<float>(A, evec2, eval[1], evec1);

      if (eval[1] < eval[0] && eval[1] < eval[2]) {
        normals_ptr[0] = evec1[0];
        normals_ptr[1] = evec1[1];
        normals_ptr[2] = evec1[2];

        return;
      }

      normals_ptr[0] = evec1[1] * evec2[2] - evec1[2] * evec2[1];
      normals_ptr[1] = evec1[2] * evec2[0] - evec1[0] * evec2[2];
      normals_ptr[2] = evec1[0] * evec2[1] - evec1[1] * evec2[0];

      return;
    } else {
      ComputeEigenvector0<float>(A, eval[0], evec0);

      if (eval[0] < eval[1] && eval[0] < eval[2]) {
        normals_ptr[0] = evec0[0];
        normals_ptr[1] = evec0[1];
        normals_ptr[2] = evec0[2];
        return;
      }

      ComputeEigenvector1<float>(A, evec0, eval[1], evec1);

      if (eval[1] < eval[0] && eval[1] < eval[2]) {
        normals_ptr[0] = evec1[0];
        normals_ptr[1] = evec1[1];
        normals_ptr[2] = evec1[2];
        return;
      }

      normals_ptr[0] = evec0[1] * evec1[2] - evec0[2] * evec1[1];
      normals_ptr[1] = evec0[2] * evec1[0] - evec0[0] * evec1[2];
      normals_ptr[2] = evec0[0] * evec1[1] - evec0[1] * evec1[0];
      return;
    }
  } else {
    if (covariance_ptr[0] < covariance_ptr[4] &&
        covariance_ptr[0] < covariance_ptr[8]) {
      normals_ptr[0] = 1.0;
      normals_ptr[1] = 0.0;
      normals_ptr[2] = 0.0;
      return;
    } else if (covariance_ptr[0] < covariance_ptr[4] &&
               covariance_ptr[0] < covariance_ptr[8]) {
      normals_ptr[0] = 0.0;
      normals_ptr[1] = 1.0;
      normals_ptr[2] = 0.0;
      return;
    } else {
      normals_ptr[0] = 0.0;
      normals_ptr[1] = 0.0;
      normals_ptr[2] = 1.0;
      return;
    }
  }
}

void EstimateNormalsFromCovariancesCPU(const torch::Tensor &covariances,
                                       torch::Tensor &normals,
                                       const bool has_normals) {
  int64_t n = covariances.size(0);

  const float *covariances_ptr = covariances.data_ptr<float>();
  float *normals_ptr = normals.data_ptr<float>();

  for (int i = 0; i < n; ++i) {
    int32_t covariances_offset = 9 * i;
    int32_t normals_offset = 3 * i;
    float normals_output[3] = {0};
    EstimatePointWiseNormalsWithFastEigen3x3(
        covariances_ptr + covariances_offset, normals_output);

    if ((normals_output[0] * normals_output[0] +
         normals_output[1] * normals_output[1] +
         normals_output[2] * normals_output[2]) == 0.0 &&
        !has_normals) {
      normals_output[0] = 0.0;
      normals_output[1] = 0.0;
      normals_output[2] = 1.0;
    }
    if (has_normals) {
      if ((normals_ptr[normals_offset] * normals_output[0] +
           normals_ptr[normals_offset + 1] * normals_output[1] +
           normals_ptr[normals_offset + 2] * normals_output[2]) < 0.0) {
        normals_output[0] *= -1;
        normals_output[1] *= -1;
        normals_output[2] *= -1;
      }
    }

    normals_ptr[normals_offset] = normals_output[0];
    normals_ptr[normals_offset + 1] = normals_output[1];
    normals_ptr[normals_offset + 2] = normals_output[2];
  }
}

void EstimateNormals(const torch::Tensor &points, const int max_knn,
                     const double radius) {
  const torch::TensorOptions opts = torch::TensorOptions()
                                        .dtype(points.scalar_type())
                                        .device(points.device());
  torch::Tensor normals = torch::zeros({points.size(0), 3}, opts);
  torch::Tensor covariances = torch::zeros({points.size(0), 3, 3}, opts);

  if (radius > 0 && max_knn > 0) {
    std::cout << "Using Hybrid Search for computing covariances" << std::endl;
    EstimateCovariancesUsingHybridSearchCPU(points.contiguous(), covariances,
                                            radius, max_knn);
  } else if (max_knn > 0 && radius < 0) {
    std::cout << "Using KNN Search for computing covariances" << std::endl;
    EstimateCovariancesUsingKNNSearchCPU(points.contiguous(), covariances,
                                         max_knn);
  } else if (max_knn < 0 && radius > 0) {
    std::cout << "Using Radius Search for computing covariances" << std::endl;
    EstimateCovariancesUsingRadiusSearchCPU(points.contiguous(), covariances,
                                            radius);
  }

  EstimateNormalsFromCovariancesCPU(covariances, normals, false);

  return;
}
