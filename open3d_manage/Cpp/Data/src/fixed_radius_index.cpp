#include "Data/fixed_radius_index.h"

FixedRadiusIndex::FixedRadiusIndex(){};

FixedRadiusIndex::FixedRadiusIndex(const torch::Tensor &dataset_points,
                                   double radius) {
  SetTensorData(dataset_points, radius);
};

FixedRadiusIndex::FixedRadiusIndex(const torch::Tensor &dataset_points,
                                   double radius,
                                   const torch::Dtype &index_dtype) {
  SetTensorData(dataset_points, radius, index_dtype);
};

FixedRadiusIndex::~FixedRadiusIndex(){};

bool FixedRadiusIndex::SetTensorData(const torch::Tensor &dataset_points,
                                     double radius,
                                     const torch::Dtype &index_dtype) {
  const int64_t num_dataset_points = dataset_points.size(0);
  const torch::TensorOptions opts = torch::TensorOptions().dtype(torch::kInt64);
  torch::Tensor points_row_splits =
      torch::zeros({0, num_dataset_points, 2}, opts);
  return SetTensorData(dataset_points, points_row_splits, radius, index_dtype);
}

bool FixedRadiusIndex::SetTensorData(const torch::Tensor &dataset_points,
                                     const torch::Tensor &points_row_splits,
                                     double radius,
                                     const torch::Dtype &index_dtype) {
  if (radius <= 0) {
    std::cout << "radius should be positive." << std::endl;
  }
  if (dataset_points.size(0) != points_row_splits[-1].item<int64_t>()) {
    std::cout << "dataset_points and points_row_splits have incompatible "
                 "shapes."
              << std::endl;
  }

  dataset_points_ = dataset_points.contiguous();
  points_row_splits_ = points_row_splits.contiguous();
  index_dtype_ = index_dtype;

  const int64_t num_dataset_points = GetDatasetSize();
  const int64_t num_batch = points_row_splits.size(0) - 1;
  const torch::Device device = GetDevice();
  const torch::Dtype dtype = GetDtype();

  std::vector<uint32_t> hash_table_splits(num_batch + 1, 0);
  for (int i = 0; i < num_batch; ++i) {
    int64_t num_dataset_points_i = points_row_splits_[i + 1].item<int64_t>() -
                                   points_row_splits_[i].item<int64_t>();
    int64_t hash_table_size = std::min<int64_t>(
        std::max<int64_t>(hash_table_size_factor * num_dataset_points_i, 1),
        max_hash_tabls_size);
    hash_table_splits[i + 1] = hash_table_splits[i] + (uint32_t)hash_table_size;
  }

  hash_table_splits_ =
      torch::zeros({long(hash_table_splits.size()), num_batch + 1},
                   torch::TensorOptions().dtype(torch::kUInt32));
  hash_table_index_ =
      torch::empty({num_dataset_points},
                   torch::TensorOptions().dtype(torch::kUInt32).device(device));
  hash_table_cell_splits_ =
      torch::empty({hash_table_splits.back() + 1},
                   torch::TensorOptions().dtype(torch::kUInt32).device(device));

#define BUILD_PARAMETERS                                                       \
  dataset_points_, radius, points_row_splits_, hash_table_splits_,             \
      hash_table_index_, hash_table_cell_splits_

#define CALL_BUILD(type, fn)                                                   \
  if (Dtype::FromType<type>() == dtype) {                                      \
    fn<type>(BUILD_PARAMETERS);                                                \
    return true;                                                               \
  }

  return false;
};

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
FixedRadiusIndex::SearchRadius(const torch::Tensor &query_points, double radius,
                               bool sort) const {
  const int64_t num_query_points = query_points.size(0);
  torch::Tensor queries_row_splits = torch::zeros(
      {0, num_query_points, 2}, torch::TensorOptions().dtype(torch::kInt64));
  return SearchRadius(query_points, queries_row_splits, radius, sort);
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor>
FixedRadiusIndex::SearchRadius(const torch::Tensor &query_points,
                               const torch::Tensor &queries_row_splits,
                               double radius, bool sort) const {
  const torch::Dtype dtype = GetDtype();
  const torch::Dtype index_dtype = GetIndexDtype();
  const torch::Device device = GetDevice();

  const int64_t num_query_points = query_points.size(0);
  if (num_query_points != queries_row_splits[-1].item<int64_t>()) {
    std::cout << "query_points and queries_row_splits have incompatible "
                 "shapes."
              << std::endl;
  }

  if (radius <= 0) {
    std::cout << "radius should be positive." << std::endl;
  }

  torch::Tensor query_points_ = query_points.contiguous();
  torch::Tensor queries_row_splits_ = queries_row_splits.contiguous();

  torch::Tensor neighbors_index, neighbors_distance;
  torch::Tensor neighbors_row_splits =
      torch::zeros({num_query_points + 1},
                   torch::TensorOptions().dtype(torch::kInt64).device(device));

#define RADIUS_PARAMETERS                                                      \
  dataset_points_, query_points_, radius, points_row_splits_,                  \
      queries_row_splits_, hash_table_splits_, hash_table_index_,              \
      hash_table_cell_splits_, Metric::L2, false, true, sort, neighbors_index, \
      neighbors_row_splits, neighbors_distance

  FixedRadiusSearchCPU(RADIUS_PARAMETERS);

  return std::make_tuple(neighbors_index, neighbors_distance,
                         neighbors_row_splits.toType(index_dtype));
};

std::tuple<Tensor, Tensor, Tensor>
FixedRadiusIndex::SearchHybrid(const Tensor &query_points, double radius,
                               int max_knn) const {
  const int64_t num_query_points = query_points.GetShape()[0];
  Tensor queries_row_splits(std::vector<int64_t>({0, num_query_points}), {2},
                            Int64);
  return SearchHybrid(query_points, queries_row_splits, radius, max_knn);
}

std::tuple<Tensor, Tensor, Tensor>
FixedRadiusIndex::SearchHybrid(const Tensor &query_points,
                               const Tensor &queries_row_splits, double radius,
                               int max_knn) const {
  const Dtype dtype = GetDtype();
  const Dtype index_dtype = GetIndexDtype();
  const Device device = GetDevice();

  // Check device and dtype.
  AssertTensorDevice(query_points, device);
  AssertTensorDtype(query_points, dtype);
  AssertTensorDevice(queries_row_splits, Device("CPU:0"));
  AssertTensorDtype(queries_row_splits, Int64);

  // Check shape.
  AssertTensorShape(query_points, {utility::nullopt, GetDimension()});
  AssertTensorShape(queries_row_splits, points_row_splits_.GetShape());

  const int64_t num_query_points = query_points.GetShape()[0];
  if (num_query_points != queries_row_splits[-1].Item<int64_t>()) {
    std::cout << "query_points and queries_row_splits have incompatible "
                 "shapes."
              << std::endl;
  }

  if (radius <= 0) {
    std::cout << "radius should be positive." << std::endl;
  }

  Tensor query_points_ = query_points.Contiguous();
  Tensor queries_row_splits_ = queries_row_splits.Contiguous();

  Tensor neighbors_index, neighbors_distance, neighbors_count;

#define HYBRID_PARAMETERS                                                      \
  dataset_points_, query_points_, radius, max_knn, points_row_splits_,         \
      queries_row_splits_, hash_table_splits_, hash_table_index_,              \
      hash_table_cell_splits_, Metric::L2, neighbors_index, neighbors_count,   \
      neighbors_distance

  if (device.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
    DISPATCH_FLOAT_INT_DTYPE_TO_TEMPLATE(dtype, index_dtype, [&]() {
      HybridSearchCUDA<scalar_t, int_t>(HYBRID_PARAMETERS);
    });
#else
    std::cout << "-DBUILD_CUDA_MODULE=OFF. Please compile Open3d with "
                 "-DBUILD_CUDA_MODULE=ON."
              << std::endl;
#endif
  } else {
    DISPATCH_FLOAT_INT_DTYPE_TO_TEMPLATE(dtype, index_dtype, [&]() {
      HybridSearchCPU<scalar_t, int_t>(HYBRID_PARAMETERS);
    });
  }

  return std::make_tuple(neighbors_index.View({num_query_points, max_knn}),
                         neighbors_distance.View({num_query_points, max_knn}),
                         neighbors_count.View({num_query_points}));
}
