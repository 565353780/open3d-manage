#include "filter.h"
#include "noise_dataset_loader.h"
#include <iostream>
#include <string>

int main() {
  // dataset params
  const std::string dataset_root_folder_path = "/home/chli/Dataset/";

  // super params
  const float sigma_d = 200;
  const float sigma_n = 2000;
  const int curvature_knn_num = 10;
  const int filter_knn_num = 40;
  const bool need_smooth = true;

  // call data from dataset
  NoiseDatasetLoader noise_dataset_loader(dataset_root_folder_path);
  const std::vector<std::string> shape_id_vec = noise_dataset_loader.getShapeIdVec();
  const std::string shape_id = shape_id_vec[0];
  const std::string noise_type = "Gauss";
  std::unordered_map<std::string, std::string> params;
  params["sample_point_num"] = "10000"; // 10000, 100000

  if (noise_type == "Random"){
    params["strength"] = "0.005"; // 0.005, 0.01, 0.02
  }
  else if (noise_type == "Gauss"){
    params["mean"] = "0.0"; // 0.0
    params["sigma"] = "0.005"; // 0.005, 0.01, 0.02
  }
  else if (noise_type == "Impulse"){
    params["strength"] = "0.02"; // 0.02, 0.04, 0.08
    params["probability"] = "0.1"; // 0.1
  }
  else{
    std::cout << "noise type not defined!" << std::endl;
    return -1;
  }

  const std::string shape_file_path = noise_dataset_loader.getShapeFilePath(shape_id, noise_type, params);
  std::cout << shape_file_path << std::endl;

  // input point cloud [x1, y1, z1, x2, y2, z2, ...]
  std::vector<float> points;
  points.resize(3000);
  for (int i = 0; i < 1000; ++i) {
    points[3 * i] = 1.0 * i;
    points[3 * i + 1] = 2.0 * i;
    points[3 * i + 2] = 3.0 * i;
  }

  // filter method call
  const std::vector<float> denoised_points = toDenoisedPts(
      points, sigma_d, sigma_n, curvature_knn_num, filter_knn_num, need_smooth);

  // result output demo
  std::cout << "input points size: " << points.size() / 3 << std::endl;
  std::cout << "denoised points size: " << denoised_points.size() / 3
            << std::endl;

  std::cout << "from: [" << points[0] << "," << points[1] << "," << points[2]
            << "]" << std::endl;
  std::cout << "to: [" << denoised_points[0] << "," << denoised_points[1] << ","
            << denoised_points[2] << "]" << std::endl;

  return 1;
}
