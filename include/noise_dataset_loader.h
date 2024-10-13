#pragma once

#include <string>
#include <unordered_map>
#include <vector>

class NoiseDatasetLoader {
public:
  NoiseDatasetLoader(const std::string &dataset_root_folder_path);
  NoiseDatasetLoader(const std::string &dataset_root_folder_path, const std::string &dataset_done_folder_path);

  const bool isValid();

  const std::vector<std::string> getShapeIdVec();

  const std::string getNoisePcdFilePath(
      const std::string &shape_id,
      const std::string &noise_type,
      const std::unordered_map<std::string, std::string> &params);

  const std::vector<float> getNoisePoints(
      const std::string &shape_id,
      const std::string &noise_type,
      const std::unordered_map<std::string, std::string> &params);

  const std::string getCurrentProcessNoiseFile(){
    return current_process_noise_file_;
  }

  const std::string getPostProcessNoiseFile(){
    return post_process_noise_file_;
  }

  const std::string getCurrentProcessMeshFile(){
    return current_process_mesh_file_;
  }

  const std::string getPostProcessMeshFile(){
    return post_process_mesh_file_;
  }

private:
  std::string dataset_root_folder_path_ = "";

  std::string source_mesh_folder_path_ = "";
  std::string noise_pcd_folder_path_ = "";

  std::string source_mesh_folder_path_test_ = "";
  std::string noise_pcd_folder_path_test_ = "";

  std::string current_process_noise_file_ = ""; 
  std::string post_process_noise_file_ = "";
  std::string current_process_mesh_file_ = "";
  std::string post_process_mesh_file_ = "";
  
};
