#pragma once

#include <string>
#include <unordered_map>
#include <vector>

class NoiseDatasetLoader {
public:
  NoiseDatasetLoader(const std::string &dataset_root_folder_path);

  const bool isValid();

  const std::vector<std::string> getShapeIdVec();

  const std::string getShapeFilePath(
      const std::string &shape_id,
      const std::string &noise_type,
      const std::unordered_map<std::string, std::string> &params);

private:
  std::string dataset_root_folder_path_ = "";

  std::string source_mesh_folder_path_ = "";
  std::string noise_pcd_folder_path_ = "";
};
