#include "io.h"
#include <open3d/Open3D.h>
#include <filesystem>

const bool savePointsAsPcd(const std::vector<float> &points, const std::string &save_file_path, const bool &overwrite){
  if (std::filesystem::exists(save_file_path)){
    if (!overwrite){
      return true;
    }

    std::filesystem::remove(save_file_path);
  }

  const std::string save_folder_path = std::filesystem::path(save_file_path).parent_path();
  if (!std::filesystem::exists(save_folder_path)) {
    std::filesystem::create_directories(save_folder_path);
  }

  const int point_num = int(points.size() / 3);

  open3d::geometry::PointCloud pcd;
  pcd.points_.reserve(point_num);

  for (int i = 0; i < point_num; ++i) {
    pcd.points_.emplace_back(
        Eigen::Vector3d(points[3 * i], points[3 * i + 1], points[3 * i + 2]));
  }

  open3d::io::WritePointCloudOption option;
  option.write_ascii = open3d::io::WritePointCloudOption::IsAscii::Ascii;

  open3d::io::WritePointCloud(save_file_path, pcd, option);

  return true;
}

const std::vector<float> loadPointsFromFile(const std::string &file_path){
  std::vector<float> points;

  if (!std::filesystem::exists(file_path)){
    std::cout << "[ERROR][io::loadPointsFromFile]" << std::endl;
    std::cout << "\t file not exist!" << std::endl;
    std::cout << "\t file_path : " << file_path << std::endl;
    return points;
  }

  open3d::geometry::PointCloud pcd;
  if (!open3d::io::ReadPointCloud(file_path, pcd)){
    std::cout << "[ERROR][io::loadPointsFromFile]" << std::endl;
    std::cout << "\t ReadPointCloud failed!" << std::endl;
    std::cout << "\t file_path : " << file_path << std::endl;
    return points;
  }

  points.reserve(pcd.points_.size() * 3);
  for (auto point : pcd.points_){
    points.emplace_back(point[0]);
    points.emplace_back(point[1]);
    points.emplace_back(point[2]);
  }

  return points;
}
