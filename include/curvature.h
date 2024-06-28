#pragma once

#include "NumCpp.hpp"

const nc::NdArray<float> toCurvaturesByFit(const nc::NdArray<float> &points,
                                           const int &knn_num);
