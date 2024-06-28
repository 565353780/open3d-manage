#pragma once

#include "NumCpp.hpp"

const nc::NdArray<int64_t> toKNNIdxs(const nc::NdArray<float> &points,
                                     const int &knn_num);
