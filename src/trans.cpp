#include "trans.h"

nc::NdArray<float> toArray(const std::vector<float> &vec_data) {
  nc::NdArray<float> arr_data =
      nc::NdArray<float>(vec_data.data(), vec_data.size());

  return arr_data;
}

std::vector<float> toVector(const nc::NdArray<float> &arr_data) {
  return arr_data.toStlVector();
}
