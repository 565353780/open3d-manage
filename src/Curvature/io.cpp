#include "Curvature/io.h"
#include <filesystem>

std::shared_ptr<open3d::geometry::TriangleMesh>
loadMeshFile(const std::string &mesh_file_path) {
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
      std::make_shared<open3d::geometry::TriangleMesh>();

  if (!std::filesystem::exists(mesh_file_path)) {
    std::cout << "[ERROR][CurvatureEstimator::toMeshTotalCurvature]"
              << std::endl;
    std::cout << "\t mesh file not exist!" << std::endl;
    std::cout << "\t mesh_file_path: " << mesh_file_path << std::endl;
    return mesh_ptr;
  }

  if (!open3d::io::ReadTriangleMesh(mesh_file_path, *mesh_ptr)) {
    std::cout << "[ERROR][CurvatureEstimator::toMeshTotalCurvature]"
              << std::endl;
    std::cout << "\t ReadTriangleMesh failed!" << std::endl;
    std::cout << "\t mesh_file_path: " << mesh_file_path << std::endl;
    // mesh_ptr.reset();
    return mesh_ptr;
  }

  return mesh_ptr;
}

std::shared_ptr<open3d::geometry::PointCloud>
loadPcdFile(const std::string &pcd_file_path) {
  std::shared_ptr<open3d::geometry::PointCloud> point_cloud_ptr =
      std::make_shared<open3d::geometry::PointCloud>();

  if (!std::filesystem::exists(pcd_file_path)) {
    std::cout << "[ERROR][CurvatureEstimator::toPcdTotalCurvature]"
              << std::endl;
    std::cout << "\t pcd file not exist!" << std::endl;
    std::cout << "\t pcd_file_path: " << pcd_file_path << std::endl;
    // point_cloud_ptr.reset();
    return point_cloud_ptr;
  }

  open3d::io::ReadPointCloud(pcd_file_path, *point_cloud_ptr);

  return point_cloud_ptr;
}
