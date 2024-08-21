#include "noise_dataset_loader.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

NoiseDatasetLoader::NoiseDatasetLoader(const std::string &dataset_root_folder_path){
  dataset_root_folder_path_ = dataset_root_folder_path;

  source_mesh_folder_path_ = dataset_root_folder_path_ + "ManifoldMesh/ShapeNet/02691156/";
  noise_pcd_folder_path_ = dataset_root_folder_path_ + "ManifoldMesh_NoisePcd/ShapeNet/02691156/";

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
  std::vector<std::string> shape_id_vec;

  if (!isValid()){
    return shape_id_vec;
  }

  for (const auto& entry : std::filesystem::directory_iterator(noise_pcd_folder_path_)) {
    if (!entry.is_directory()) {
      continue;
    }

    const std::string shape_id = entry.path().filename();

    shape_id_vec.emplace_back(shape_id);
  }

  return shape_id_vec;
}

const std::string NoiseDatasetLoader::getShapeFilePath(
      const std::string &shape_id,
      const std::string &noise_type,
      const std::unordered_map<std::string, std::string> &params){
  std::string shape_file_path = "";

  const std::string shape_noise_pcd_folder_path = noise_pcd_folder_path_ + shape_id + "/" + noise_type + "/";

  if (!std::filesystem::exists(shape_noise_pcd_folder_path)) {
    std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
    std::cout << "\t shape noise pcd folder not exist!" << std::endl;
    std::cout << "\t shape_noise_pcd_folder_path : " << shape_noise_pcd_folder_path << std::endl;
    return shape_file_path;
  }

  std::string noise_pcd_file_name = "";

  auto it = params.find("sample_point_num");
  if (it == params.end()){
    std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
    std::cout << "\t param sample point num not found!" << std::endl;
    return shape_file_path;
  }

  noise_pcd_file_name += "sample-" + it->second;

  if (noise_type == "Random"){
    it = params.find("strength");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
      std::cout << "\t param strength not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_strength-" + it->second;
  }
  else if (noise_type == "Gauss") {
    it = params.find("mean");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
      std::cout << "\t param mean not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_mean-" + it->second;

    it = params.find("sigma");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
      std::cout << "\t param sigma not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_sigma-" + it->second;
  }
  else if (noise_type == "Impulse") {
    it = params.find("strength");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
      std::cout << "\t param strength not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_strength-" + it->second;

    it = params.find("probability");

    if (it == params.end()){
      std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
      std::cout << "\t param probability not found for " + noise_type + " noise!" << std::endl;
      return shape_file_path;
    }

    noise_pcd_file_name += "_probability-" + it->second;
  }
  else{
    std::cout << "[ERROR][NoiseDatasetLoader::getShapeFilePath]" << std::endl;
    std::cout << "\t noise type not valid!" << std::endl;
    std::cout << "\t noise_type : " << noise_type << std::endl;
    std::cout << "\t valid noise types : Random, Gauss, Impulse" << std::endl;
  }

  noise_pcd_file_name += ".ply";

  shape_file_path = shape_noise_pcd_folder_path + noise_pcd_file_name;

  return shape_file_path;
}
