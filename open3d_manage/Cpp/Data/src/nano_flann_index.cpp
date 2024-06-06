#include "Data/nano_flann_index.h"
#include <nanoflann.hpp>

template <int METRIC, class TReal, class TIndex>
struct NanoFlannIndexHolder : NanoFlannIndexHolderBase {
  struct DataAdaptor {
    DataAdaptor(size_t dataset_size, int dimension, const float *const data_ptr)
        : dataset_size_(dataset_size), dimension_(dimension),
          data_ptr_(data_ptr) {}

    inline size_t kdtree_get_point_count() const { return dataset_size_; }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
      return data_ptr_[idx * dimension_ + dim];
    }

    template <class BBOX> bool kdtree_get_bbox(BBOX &) const { return false; }

    size_t dataset_size_ = 0;
    int dimension_ = 0;
    const float *const data_ptr_;
  };

  template <int M, typename fake = void> struct SelectNanoflannAdaptor {};

  template <typename fake> struct SelectNanoflannAdaptor<L2, fake> {
    typedef nanoflann::L2_Adaptor<float, DataAdaptor, float> adaptor_t;
  };

  template <typename fake> struct SelectNanoflannAdaptor<L1, fake> {
    typedef nanoflann::L1_Adaptor<float, DataAdaptor, float> adaptor_t;
  };

  typedef nanoflann::KDTreeSingleIndexAdaptor<
      typename SelectNanoflannAdaptor<METRIC>::adaptor_t, DataAdaptor, -1,
      TIndex>
      KDTree_t;

  NanoFlannIndexHolder(size_t dataset_size, int dimension,
                       const float *data_ptr) {
    adaptor_.reset(new DataAdaptor(dataset_size, dimension, data_ptr));
    index_.reset(new KDTree_t(dimension, *adaptor_.get()));
    index_->buildIndex();
  }

  std::unique_ptr<KDTree_t> index_;
  std::unique_ptr<DataAdaptor> adaptor_;
};

template <class T, class TIndex, int METRIC>
void _BuildKdTree(size_t num_points, const T *const points, size_t dimension,
                  NanoFlannIndexHolderBase **holder) {
  *holder = new NanoFlannIndexHolder<METRIC, T, TIndex>(num_points, dimension,
                                                        points);
};

template <class T, class TIndex>
std::unique_ptr<NanoFlannIndexHolderBase>
BuildKdTree(size_t num_points, const float *const points, size_t dimension,
            const Metric metric) {
  NanoFlannIndexHolderBase *holder = nullptr;
#define FN_PARAMETERS num_points, points, dimension, &holder

#define CALL_TEMPLATE(METRIC)                                                  \
  if (METRIC == metric) {                                                      \
    _BuildKdTree<T, TIndex, METRIC>(FN_PARAMETERS);                            \
  }

#define CALL_TEMPLATE2                                                         \
  CALL_TEMPLATE(L1)                                                            \
  CALL_TEMPLATE(L2)

  CALL_TEMPLATE2

#undef CALL_TEMPLATE
#undef CALL_TEMPLATE2

#undef FN_PARAMETERS
  return std::unique_ptr<NanoFlannIndexHolderBase>(holder);
};

NanoFlannIndex::NanoFlannIndex(){};

NanoFlannIndex::NanoFlannIndex(const torch::Tensor &dataset_points) {
  SetTensorData(dataset_points);
};

NanoFlannIndex::NanoFlannIndex(const torch::Tensor &dataset_points,
                               const torch::Dtype &index_dtype) {
  SetTensorData(dataset_points, index_dtype);
};

NanoFlannIndex::~NanoFlannIndex(){};

bool NanoFlannIndex::SetTensorData(const torch::Tensor &dataset_points,
                                   const torch::Dtype &index_dtype) {
  if (dataset_points.sizes().size() != 2) {
    std::cout << "dataset_points must be 2D matrix, with shape "
                 "{n_dataset_points, d}."
              << std::endl;
  }

  dataset_points_ = dataset_points.contiguous();
  index_dtype_ = index_dtype;
  holder_ = BuildKdTree<float, int32_t>(
      dataset_points_.size(0), dataset_points_.data_ptr<float>(),
      dataset_points_.size(1), /* metric */ L2);
  return true;
};

std::pair<torch::Tensor, torch::Tensor>
NanoFlannIndex::SearchKnn(const torch::Tensor &query_points, int knn) const {
  const torch::Dtype dtype = GetDtype();
  const torch::Device device = GetDevice();
  const torch::Dtype index_dtype = GetIndexDtype();

  if (knn <= 0) {
    std::cout << "knn should be larger than 0." << std::endl;

    return std::pair<torch::Tensor, torch::Tensor>();
  }

  const int64_t num_neighbors = std::min(static_cast<int64_t>(GetDatasetSize()),
                                         static_cast<int64_t>(knn));
  const int64_t num_query_points = query_points.size(0);

  torch::Tensor indices, distances;
  const torch::TensorOptions opts = torch::TensorOptions().dtype(torch::kInt64);
  torch::Tensor neighbors_row_splits =
      torch::zeros({num_query_points + 1}, opts);

  DISPATCH_FLOAT_INT_DTYPE_TO_TEMPLATE(dtype, index_dtype, [&]() {
    const torch::Tensor query_contiguous = query_points.contiguous();
    NeighborSearchAllocator<scalar_t, int_t> output_allocator(device);

    impl::KnnSearchCPU<scalar_t, int_t>(
        holder_.get(), neighbors_row_splits.GetDataPtr<int64_t>(),
        dataset_points_.GetShape(0), dataset_points_.GetDataPtr<scalar_t>(),
        query_contiguous.GetShape(0), query_contiguous.GetDataPtr<scalar_t>(),
        query_contiguous.GetShape(1), num_neighbors, /* metric */ L2,
        /* ignore_query_point */ false,
        /* return_distances */ true, output_allocator);
    indices = output_allocator.NeighborsIndex();
    distances = output_allocator.NeighborsDistance();
    indices = indices.View({num_query_points, num_neighbors});
    distances = distances.View({num_query_points, num_neighbors});
  });
  return std::make_pair(indices, distances);
};

std::tuple<Tensor, Tensor, Tensor>
NanoFlannIndex::SearchRadius(const Tensor &query_points, const Tensor &radii,
                             bool sort) const {
  const Dtype dtype = GetDtype();
  const Device device = GetDevice();
  const Dtype index_dtype = GetIndexDtype();

  core::AssertTensorDevice(query_points, device);
  core::AssertTensorDevice(radii, device);
  core::AssertTensorDtype(query_points, dtype);
  core::AssertTensorDtype(radii, dtype);

  // Check shapes.
  int64_t num_query_points = query_points.GetShape(0);
  AssertTensorShape(query_points, {utility::nullopt, GetDimension()});
  AssertTensorShape(radii, {num_query_points});

  // Check if the radii has negative values.
  Tensor below_zero = radii.Le(0);
  if (below_zero.Any().Item<bool>()) {
    std::cout << "radius should be larger than 0." << std::endl;
  }

  Tensor indices, distances;
  Tensor neighbors_row_splits = Tensor({num_query_points + 1}, Int64);
  DISPATCH_FLOAT_INT_DTYPE_TO_TEMPLATE(dtype, index_dtype, [&]() {
    const Tensor query_contiguous = query_points.Contiguous();
    NeighborSearchAllocator<scalar_t, int_t> output_allocator(device);

    impl::RadiusSearchCPU<scalar_t, int_t>(
        holder_.get(), neighbors_row_splits.GetDataPtr<int64_t>(),
        dataset_points_.GetShape(0), dataset_points_.GetDataPtr<scalar_t>(),
        query_contiguous.GetShape(0), query_contiguous.GetDataPtr<scalar_t>(),
        query_contiguous.GetShape(1), radii.GetDataPtr<scalar_t>(),
        /* metric */ L2,
        /* ignore_query_point */ false, /* return_distances */ true,
        /* normalize_distances */ false, sort, output_allocator);
    indices = output_allocator.NeighborsIndex();
    distances = output_allocator.NeighborsDistance();
  });

  return std::make_tuple(indices, distances,
                         neighbors_row_splits.To(index_dtype_));
};

std::tuple<Tensor, Tensor, Tensor>
NanoFlannIndex::SearchRadius(const Tensor &query_points, double radius,
                             bool sort) const {
  const int64_t num_query_points = query_points.GetShape()[0];
  const Dtype dtype = GetDtype();
  std::tuple<Tensor, Tensor, Tensor> result;
  DISPATCH_FLOAT_DTYPE_TO_TEMPLATE(dtype, [&]() {
    Tensor radii(std::vector<scalar_t>(num_query_points, (scalar_t)radius),
                 {num_query_points}, dtype);
    result = SearchRadius(query_points, radii, sort);
  });
  return result;
};

std::tuple<Tensor, Tensor, Tensor>
NanoFlannIndex::SearchHybrid(const Tensor &query_points, double radius,
                             int max_knn) const {
  const Device device = GetDevice();
  const Dtype dtype = GetDtype();
  const Dtype index_dtype = GetIndexDtype();

  AssertTensorDevice(query_points, device);
  AssertTensorDtype(query_points, dtype);
  AssertTensorShape(query_points, {utility::nullopt, GetDimension()});

  if (max_knn <= 0) {
    std::cout << "max_knn should be larger than 0." << std::endl;
  }
  if (radius <= 0) {
    std::cout << "radius should be larger than 0." << std::endl;
  }

  int64_t num_query_points = query_points.GetShape(0);

  Tensor indices, distances, counts;
  DISPATCH_FLOAT_INT_DTYPE_TO_TEMPLATE(dtype, index_dtype, [&]() {
    const Tensor query_contiguous = query_points.Contiguous();
    NeighborSearchAllocator<scalar_t, int_t> output_allocator(device);

    impl::HybridSearchCPU<scalar_t, int_t>(
        holder_.get(), dataset_points_.GetShape(0),
        dataset_points_.GetDataPtr<scalar_t>(), query_contiguous.GetShape(0),
        query_contiguous.GetDataPtr<scalar_t>(), query_contiguous.GetShape(1),
        static_cast<scalar_t>(radius), max_knn,
        /* metric*/ L2, /* ignore_query_point */ false,
        /* return_distances */ true, output_allocator);

    indices =
        output_allocator.NeighborsIndex().View({num_query_points, max_knn});
    distances =
        output_allocator.NeighborsDistance().View({num_query_points, max_knn});
    counts = output_allocator.NeighborsCount();
  });
  return std::make_tuple(indices, distances, counts);
}
