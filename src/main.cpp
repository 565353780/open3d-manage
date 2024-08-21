#include "filter.h"
#include "noise_dataset_loader.h"
#include <iostream>
#include <limits>
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

  const std::string shape_file_path = noise_dataset_loader.getNoisePcdFilePath(shape_id, noise_type, params);
  std::cout << shape_file_path << std::endl;

  // input point cloud [x1, y1, z1, x2, y2, z2, ...]
  const std::vector<float> points = noise_dataset_loader.getNoisePoints(shape_id, noise_type, params);
  std::cout << points[0] << std::endl;
  std::cout << points[1] << std::endl;
  std::cout << points[2] << std::endl;
  std::cout << points.size() << std::endl;

  // filter method call
  const std::vector<float> denoised_points = toDenoisedPts(
      points, sigma_d, sigma_n, curvature_knn_num, filter_knn_num, need_smooth);

  // result output demo
  const int point_num = int(points.size() / 3);

  std::cout << "input points size: " << point_num << std::endl;
  std::cout << "denoised points size: " << denoised_points.size() / 3
            << std::endl;


  float min_move_dist = std::numeric_limits<float>().max();
  float mean_move_dist = 0.0;
  float max_move_dist = 0.0;
  std::vector<float> move_dists;
  move_dists.reserve(point_num);
  for (int i = 0; i < point_num; ++i){
    const float x_diff = denoised_points[3 * i] - points[3 * i];
    const float y_diff = denoised_points[3 * i + 1] - points[3 * i + 1];
    const float z_diff = denoised_points[3 * i + 2] - points[3 * i + 2];

    const float move_dist = x_diff * x_diff + y_diff * y_diff + z_diff * z_diff;

    move_dists.emplace_back(move_dist);

    if (move_dist < min_move_dist){
      min_move_dist = move_dist;
    }

    if (move_dist > max_move_dist){
      max_move_dist = move_dist;
    }

    mean_move_dist += move_dist;
  }
  mean_move_dist /= move_dists.size();

  std::cout << "move dist : min " << min_move_dist << ", mean " << mean_move_dist << ", max " << max_move_dist << std::endl;

  return 1;
}
