#include "noise_dataset_loader.h"
#include <filesystem>
#include <iostream>
#include <open3d/geometry/PointCloud.h>
#include <open3d/io/PointCloudIO.h>
#include <string>
#include <vector>
#include <open3d/Open3D.h>

NoiseDatasetLoader::NoiseDatasetLoader(const std::string &dataset_root_folder_path){
  dataset_root_folder_path_ = dataset_root_folder_path;

  source_mesh_folder_path_ = dataset_root_folder_path_ + "ManifoldMesh/ShapeNet/02691156/";
  noise_pcd_folder_path_ = dataset_root_folder_path_ + "ManifoldMesh_NoisePcd/ShapeNet/02691156/";

  isValid();
}

NoiseDatasetLoader::NoiseDatasetLoader(const std::string &dataset_root_folder_path, const std::string &dataset_done_folder_path){
  dataset_root_folder_path_ = dataset_root_folder_path;

  source_mesh_folder_path_ = dataset_root_folder_path_ + "ManifoldMesh/ShapeNet/02691156/";
  noise_pcd_folder_path_ = dataset_root_folder_path_ + "ManifoldMesh_NoisePcd/ShapeNet/02691156/";

  source_mesh_folder_path_test_ = dataset_done_folder_path + "ManifoldMesh/ShapeNet/02691156/";
  noise_pcd_folder_path_test_ = dataset_done_folder_path + "ManifoldMesh_NoisePcd/ShapeNet/02691156/";

  isValid();
}

const bool NoiseDatasetLoader::isValid(){
  if (dataset_root_folder_path_ == ""){
    std::cout << "[ERROR][NoiseDatasetLoader::isValid]" << std::endl;
    std::cout << "\t dataset root folder not set!" << std::endl;
    std::cout << "\t dataset_root_folder_path_ : " << dataset_root_folder_path_ << std::endl;
    return false;
  }

  if (!std::filesystem::exists(dataset_root_folder_path_)) {
    std::cout << "[ERROR][NoiseDatasetLoader::isValid]" << std::endl;
    std::cout << "\t dataset root folder not exist!" << std::endl;
    std::cout << "\t dataset_root_folder_path_ : " << dataset_root_folder_path_ << std::endl;
    return false;
  }

  if (!std::filesystem::exists(source_mesh_folder_path_)) {
    std::cout << "[ERROR][NoiseDatasetLoader::isValid]" << std::endl;
    std::cout << "\t source mesh folder not exist!" << std::endl;
    std::cout << "\t source_mesh_folder_path_ : " << source_mesh_folder_path_ << std::endl;
    return false;
  }

  if (!std::filesystem::exists(noise_pcd_folder_path_)) {
    std::cout << "[ERROR][NoiseDatasetLoader::isValid]" << std::endl;
    std::cout << "\t noise pcd folder not exist!" << std::endl;
    std::cout << "\t noise_pcd_folder_path_ : " << noise_pcd_folder_path_ << std::endl;
    return false;
  }

  return true;
}

const std::vector<std::string> NoiseDatasetLoader::getShapeIdVec(){
  //FIXME: these shapes are not valid for open3d
  std::vector<std::string> invalid_shape_id_vec;
  // invalid_shape_id_vec.emplace_back("133b74393a3349aa70c4138179d9ed97");

  std::vector<std::string> shape_id_vec;

  if (!isValid()){
    return shape_id_vec;
  }

  for (const auto& entry : std::filesystem::directory_iterator(noise_pcd_folder_path_)) {
    if (!entry.is_directory()) {
      continue;
    }

    const std::string shape_id = entry.path().filename();

    if (std::find(invalid_shape_id_vec.begin(), invalid_shape_id_vec.end(), shape_id) != invalid_shape_id_vec.end()){
      continue;
    }

    shape_id_vec.emplace_back(shape_id);
  }

  return shape_id_vec;
}

const std::string NoiseDatasetLoader::getNoisePcdFilePath(
      const std::string &shape_id,
      const std::string &noise_type,
      const std::unordered_map<std::string, std::string> &params){
  std::string shape_file_path = "";

  const std::string shape_noise_pcd_folder_path = noise_pcd_folder_path_ + shape_id + "/" + noise_type + "/";

  if (!std::filesystem::exists(shape_noise_pcd_folder_path)) {
    std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
    std::cout << "\t shape noise pcd folder not exist!" << std::endl;
    std::cout << "\t shape_noise_pcd_folder_path : " << shape_noise_pcd_folder_path << std::endl;
    return shape_file_path;
  }

  std::string noise_pcd_file_name = "";

  auto it = params.find("sample_point_num");
  if (it == params.end()){
    std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
    std::cout << "\t param sample point num not found!" << std::endl;
    return shape_file_path;
  }

  noise_pcd_file_name += "sample-" + it->second;

  if (noise_type == "Random"){
    it = params.find("strength");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
      std::cout << "\t param strength not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_strength-" + it->second;
  }
  else if (noise_type == "Gauss") {
    it = params.find("mean");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
      std::cout << "\t param mean not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_mean-" + it->second;

    it = params.find("sigma");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
      std::cout << "\t param sigma not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_sigma-" + it->second;
  }
  else if (noise_type == "Impulse") {
    it = params.find("strength");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
      std::cout << "\t param strength not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_strength-" + it->second;

    it = params.find("probability");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
      std::cout << "\t param probability not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_probability-" + it->second;
  }
  else{
    std::cout << "[ERROR][NoiseDatasetLoader::getNoisePcdFilePath]" << std::endl;
    std::cout << "\t noise type not valid!" << std::endl;
    std::cout << "\t noise_type : " << noise_type << std::endl;
    std::cout << "\t valid noise types : Random, Gauss, Impulse" << std::endl;
  }

  noise_pcd_file_name += ".ply";

  shape_file_path = shape_noise_pcd_folder_path + noise_pcd_file_name;

  current_process_noise_file_ = shape_file_path;
  post_process_noise_file_ = noise_pcd_folder_path_test_ + shape_id + "/" + noise_type + "/" + noise_pcd_file_name;

  current_process_mesh_file_ = source_mesh_folder_path_ + shape_id + ".obj";
  post_process_mesh_file_ = source_mesh_folder_path_test_ + shape_id + ".obj";

  return shape_file_path;
}

const std::vector<float> NoiseDatasetLoader::getNoisePoints(
      const std::string &shape_id,
      const std::string &noise_type,
      const std::unordered_map<std::string, std::string> &params){
  std::vector<float> shape_points;

  const std::string noise_pcd_file_path = getNoisePcdFilePath(shape_id, noise_type, params);
  if (noise_pcd_file_path == ""){
    std::cout << "[ERROR][NoiseDatasetLoader::getNoisePoints]" << std::endl;
    std::cout << "\t getNoisePcdFilePath failed!" << std::endl;
    return shape_points;
  }

  open3d::geometry::PointCloud noise_pcd;
  if (!open3d::io::ReadPointCloud(noise_pcd_file_path, noise_pcd)){
    std::cout << "[ERROR][NoiseDatasetLoader::getNoisePoints]" << std::endl;
    std::cout << "\t ReadPointCloud failed!" << std::endl;
    std::cout << "\t noise_pcd_file_path : " << noise_pcd_file_path << std::endl;
    return shape_points;
  }

  shape_points.reserve(noise_pcd.points_.size() * 3);
  for (auto point : noise_pcd.points_){
    shape_points.emplace_back(point[0]);
    shape_points.emplace_back(point[1]);
    shape_points.emplace_back(point[2]);
  }

  return shape_points;
}
