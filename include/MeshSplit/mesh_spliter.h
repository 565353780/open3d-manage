#pragma once

#include "MeshSplit/sub_mesh_manager.h"
#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <string>

class MeshSpliter {
public:
  MeshSpliter() {};
  MeshSpliter(const std::string &mesh_file_path);

  const bool
  splitMeshByFaceConnectivity(const std::string &save_painted_mesh_file_path);

  const bool splitMeshByVertexCurvature(
      const Eigen::VectorXd &mesh_curvatures,
      const float &max_merge_curvature = 10.0,
      const std::string &save_painted_mesh_file_path = "");

  const bool
  splitMeshByFaceNormal(const float &max_merge_angle = 10.0,
                        const std::string &save_painted_mesh_file_path = "");

  std::shared_ptr<open3d::geometry::TriangleMesh>
  toSubMesh(const std::set<int> &sub_mesh_face_idx_set,
            const bool &save_colors = true);

  const bool saveSubMeshes(const std::string &save_folder_path,
                           const bool &need_simplify = false,
                           const bool &overwrite = false);

  const std::unordered_map<int, std::set<int>>
  toSimplifiedSubMeshFaceIdxSetMap();

  const bool autoSplitMeshByFaceConnectivity(
      const std::string &save_folder_path,
      const std::string &save_painted_mesh_file_path = "",
      const bool &need_simplify = false, const bool &overwrite = false);

  const bool autoSplitMeshByVertexCurvature(
      const std::string &save_folder_path,
      const float &max_merge_curvature = 2000.0,
      const std::string &save_painted_mesh_file_path = "",
      const bool &need_simplify = false, const bool &overwrite = false);

  const bool autoSplitMeshByFaceNormal(
      const std::string &save_folder_path, const float &max_merge_angle = 10.0,
      const std::string &save_painted_mesh_file_path = "",
      const bool &need_simplify = false, const bool &overwrite = false);

private:
  SubMeshManager sub_mesh_manager_;
};
