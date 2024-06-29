#include "curvature.h"
#include "knn.h"

const nc::NdArray<float> toCurvaturesByFit(const nc::NdArray<float> &points,
                                           const int &knn_num) {
  const int point_num = points.shape().rows;

  const nc::NdArray<int64_t> idxs = toKNNIdxs(points, knn_num);

  nc::NdArray<float> curvatures = nc::zeros<float>(1, point_num);

  for (int i = 0; i < point_num; ++i) {
    nc::NdArray<float> A = nc::zeros<float>(knn_num, 6);
    nc::NdArray<float> b = nc::zeros<float>(knn_num, 1);

    for (int j = 0; j < knn_num; ++j) {
      const int64_t idx = idxs(i, j);
      const nc::NdArray<float> &p = points.row(idx);

      A(j, 0) = p[0] * p[0];
      A(j, 1) = p[0] * p[1];
      A(j, 2) = p[1] * p[1];
      A(j, 3) = p[0];
      A(j, 4) = p[1];
      A(j, 5) = 1.0f;

      b(j, 0) = p[2];
    }

    nc::NdArray<float> x = nc::linalg::lstsq<float>(A, b).astype<float>();

    const nc::NdArray<float> &point = points.row(i);

    const nc::NdArray<float> rx = {
        1.0f,
        0.0f,
        2.0f * float(x[0]) * point[0] + float(x[1]) * point[1] + float(x[3]),
    };
    const float rx_norm = nc::norm<float>(rx)[0];

    const nc::NdArray<float> ry = {
        0.0f,
        1.0f,
        2.0f * float(x[2]) * point[1] + float(x[1]) * point[0] + float(x[4]),
    };
    const float ry_norm = nc::norm<float>(ry)[0];

    const float rx_dot_ry = nc::dot<float>(rx, ry)[0];
    const nc::NdArray<float> rx_cross_ry = nc::cross<float>(rx, ry);

    const nc::NdArray<float> rxx = {0.0f, 0.0f, 2.0f * float(x[0])};
    const nc::NdArray<float> rxy = {0.0f, 0.0f, float(x[1])};
    const nc::NdArray<float> ryy = {0.0f, 0.0f, 2.0f * float(x[2])};

    const float k1 =
        rx_norm * rx_norm * ry_norm * ry_norm - rx_dot_ry * rx_dot_ry;
    const nc::NdArray<float> n =
        rx_cross_ry / float(nc::norm<float>(rx_cross_ry)[0]);

    const float rxy_dot_n = nc::dot<float>(rxy, n)[0];
    const float k2 = nc::dot<float>(rxx, n)[0] * nc::dot<float>(ryy, n)[0] -
                     rxy_dot_n * rxy_dot_n;

    curvatures[i] = k2 / (k1 + 1e-6);
  }

  return curvatures;
}
