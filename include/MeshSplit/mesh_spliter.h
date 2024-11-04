#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <string>

class MeshSpliter {
public:
  MeshSpliter() {};

  const std::unordered_map<int, std::set<int>> splitMeshByCurvature(
      std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
      const Eigen::VectorXd &mesh_curvatures,
      const float &max_merge_curvature = 10.0);

  std::shared_ptr<open3d::geometry::TriangleMesh>
  toSubMesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
            const std::set<int> &sub_mesh_face_idx_set,
            const bool &save_colors = true);

  const bool saveSubMeshes(
      std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
      const std::unordered_map<int, std::set<int>> &sub_mesh_face_idx_set_map,
      const std::string &save_folder_path, const bool &overwrite = false);

  const bool autoSplitMesh(const std::string &mesh_file_path,
                           const std::string &save_folder_path,
                           const float &max_merge_curvature = 2000.0,
                           const bool &overwrite = false);
};
