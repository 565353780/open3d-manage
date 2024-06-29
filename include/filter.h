#pragma once

#include "NumCpp.hpp"

const nc::NdArray<float> toFilterWeights(const nc::NdArray<float> &curvatures);

const nc::NdArray<float>
toBilateralFilterPts(const nc::NdArray<float> &points, const float &sigma_d,
                     const float &sigma_n, const int &knn_num,
                     const nc::NdArray<float> &curvature_weights,
                     const bool need_smooth = false);

const std::vector<float>
toDenoisedPts(const std::vector<float> &points, const float &sigma_d,
              const float &sigma_n, const int &curvature_knn_num,
              const int &filter_knn_num, const bool &need_smooth = false);
