#pragma once

#include "NumCpp.hpp"

const nc::NdArray<float> toNormals(const nc::NdArray<float> &points,
                                   const int &knn_num = 0,
                                   const bool &need_smooth = false);
