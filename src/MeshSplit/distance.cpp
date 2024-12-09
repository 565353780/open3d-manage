#include "MeshSplit/distance.h"
#include <filesystem>
#include <limits>

const bool toMeshVertexPcdDistanceVec(
    std::shared_ptr<open3d::geometry::TriangleMesh> &o3d_mesh_ptr,
    const std::string &pcd_file_path,
    std::vector<float> &vertex_pcd_distance_vec) {
  if (!std::filesystem::exists(pcd_file_path)) {
    std::cerr << "[ERROR][distance::toMeshVertexPcdDistanceVec]" << std::endl;
    std::cerr << "\t pcd file not exist!" << std::endl;
    std::cerr << "\t pcd_file_path: " << pcd_file_path << std::endl;

    return false;
  }

  auto pcd = open3d::geometry::PointCloud();

  if (!open3d::io::ReadPointCloud(pcd_file_path, pcd)) {
    std::cerr << "[ERROR][distance::toMeshVertexPcdDistanceVec]" << std::endl;
    std::cerr << "\t ReadPointCloud failed!" << std::endl;
    std::cerr << "\t pcd_file_path: " << pcd_file_path << std::endl;

    return false;
  }

  open3d::geometry::KDTreeFlann kdtree;
  kdtree.SetGeometry(pcd);

  vertex_pcd_distance_vec.resize(o3d_mesh_ptr->vertices_.size(),
                                 std::numeric_limits<float>::max());

  for (size_t i = 0; i < o3d_mesh_ptr->vertices_.size(); ++i) {
    const auto &point = o3d_mesh_ptr->vertices_[i];
    std::vector<int> indices(1);
    std::vector<double> distances(1);

    if (kdtree.SearchKNN(point, 1, indices, distances) > 0) {
      vertex_pcd_distance_vec[i] = float(std::sqrt(distances[0]));
    }
  }

  return true;
}
