#pragma once

#include "NumCpp.hpp"
#include <vector>

nc::NdArray<float> toArray(const std::vector<float> &vec_data);

std::vector<float> toVector(const nc::NdArray<float> &arr_data);
