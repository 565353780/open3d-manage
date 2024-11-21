#include "MeshSplit/mesh_spliter.h"
#include "Curvature/curvature_estimator.h"
#include "Curvature/io.h"
#include "Curvature/render.h"
#include "MeshSplit/idx_curvature.h"
#include "MeshSplit/sub_mesh_manager.h"
#include <algorithm>
#include <filesystem>

const std::unordered_map<int, std::set<int>> MeshSpliter::splitMeshByCurvature(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
    const Eigen::VectorXd &mesh_curvatures, const float &max_merge_curvature,
    const std::string &save_painted_mesh_file_path) {
  const int vertex_num = mesh_ptr->vertices_.size();

  if (vertex_num != mesh_curvatures.size()) {
    std::cout << "[ERROR][MeshSpliter::splitMeshByCurvature]" << std::endl;
    std::cout << "\t mesh vertex num != mesh curvatures num!" << std::endl;
    return std::unordered_map<int, std::set<int>>();
  }

  std::vector<double> curvatures_vec(mesh_curvatures.data(),
                                     mesh_curvatures.data() + vertex_num);

  std::vector<IdxCurvature> unused_curvatures;
  unused_curvatures.reserve(vertex_num);

  for (int i = 0; i < vertex_num; ++i) {
    unused_curvatures.emplace_back(IdxCurvature(i, mesh_curvatures[i]));
  }

  std::sort(
      unused_curvatures.begin(), unused_curvatures.end(),
      [](IdxCurvature a, IdxCurvature b) { return a.curvature > b.curvature; });

  SubMeshManager sub_mesh_manager(mesh_ptr);

  // 按照曲率从小到大逐点检查
  while (!unused_curvatures.empty()) {
    const IdxCurvature current_unused_idx_curvature = unused_curvatures.back();
    unused_curvatures.pop_back();

    const int current_vertex_idx = current_unused_idx_curvature.idx;

    sub_mesh_manager.addVertexIntoSubSet(current_vertex_idx, curvatures_vec,
                                         max_merge_curvature);
  }

  sub_mesh_manager.sortSubMeshIdxSetMap();

  // sub_mesh_manager.checkSubMeshState();

  if (save_painted_mesh_file_path != "") {
    const std::string save_painted_mesh_folder_path =
        std::filesystem::path(save_painted_mesh_file_path)
            .parent_path()
            .string();

    if (!std::filesystem::exists(save_painted_mesh_folder_path)) {
      std::filesystem::create_directories(save_painted_mesh_folder_path);
    }

    sub_mesh_manager.paintSubMesh();
    // sub_mesh_manager.renderSubMeshes();
    sub_mesh_manager.savePaintedMesh(save_painted_mesh_file_path, true);
  }

  return sub_mesh_manager.sub_mesh_face_idx_set_map;
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshSpliter::toSubMesh(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
    const std::set<int> &sub_mesh_face_idx_set, const bool &save_colors) {
  const std::vector<Eigen::Vector3d> &vertices = mesh_ptr->vertices_;
  const std::vector<Eigen::Vector3i> &triangles = mesh_ptr->triangles_;

  std::vector<Eigen::Vector3d> vertex_colors;
  if (save_colors) {
    vertex_colors = mesh_ptr->vertex_colors_;
  }

  std::vector<Eigen::Vector3d> sub_mesh_vertices;
  std::vector<Eigen::Vector3i> sub_mesh_triangles;
  std::vector<Eigen::Vector3d> sub_mesh_vertex_colors;
  std::unordered_map<int, int> vertex_idx_map;
  int new_vertex_index = 0;

  for (int i : sub_mesh_face_idx_set) {
    const Eigen::Vector3i triangle = triangles[i];
    Eigen::Vector3i new_triangle;

    for (int j = 0; j < 3; ++j) {
      int vertex_index = triangle[j];
      if (vertex_idx_map.find(vertex_index) == vertex_idx_map.end()) {
        vertex_idx_map[vertex_index] = new_vertex_index++;
        sub_mesh_vertices.push_back(vertices[vertex_index]);
        if (vertex_colors.size() == vertices.size()) {
          sub_mesh_vertex_colors.push_back(vertex_colors[vertex_index]);
        }
      }
      new_triangle[j] = vertex_idx_map[vertex_index];
    }
    sub_mesh_triangles.push_back(new_triangle);
  }

  std::shared_ptr<open3d::geometry::TriangleMesh> sub_mesh_ptr =
      std::make_shared<open3d::geometry::TriangleMesh>();
  sub_mesh_ptr->vertices_ = sub_mesh_vertices;
  sub_mesh_ptr->triangles_ = sub_mesh_triangles;
  if (sub_mesh_vertex_colors.size() == sub_mesh_vertices.size()) {
    sub_mesh_ptr->vertex_colors_ = sub_mesh_vertex_colors;
  }

  return sub_mesh_ptr;
}

const bool MeshSpliter::saveSubMeshes(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
    const std::unordered_map<int, std::set<int>> &sub_mesh_face_idx_set_map,
    const std::string &save_folder_path, const bool &overwrite) {
  if (std::filesystem::exists(save_folder_path)) {
    if (!overwrite) {
      return true;
    }

    std::filesystem::remove_all(save_folder_path);
  }

  std::filesystem::create_directories(save_folder_path);

  for (auto it = sub_mesh_face_idx_set_map.begin();
       it != sub_mesh_face_idx_set_map.end(); ++it) {
    const int sub_mesh_idx = it->first;
    const std::set<int> sub_mesh_face_idx_set = it->second;

    std::shared_ptr<open3d::geometry::TriangleMesh> sub_mesh_ptr =
        toSubMesh(mesh_ptr, sub_mesh_face_idx_set);

    const std::string save_file_path =
        save_folder_path + "sub_mesh_" + std::to_string(sub_mesh_idx) + ".ply";

    open3d::io::WriteTriangleMesh(save_file_path, *sub_mesh_ptr, true);
  }

  return true;
}

const bool MeshSpliter::autoSplitMesh(
    const std::string &mesh_file_path, const std::string &save_folder_path,
    const float &max_merge_curvature,
    const std::string &save_painted_mesh_file_path, const bool &overwrite) {
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
      loadMeshFile(mesh_file_path);
  if (mesh_ptr->IsEmpty()) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMesh]" << std::endl;
    std::cerr << "\t loadMeshFile failed!" << std::endl;
    std::cerr << "\t mesh_file_path: " << mesh_file_path << std::endl;
    return false;
  }

  CurvatureEstimator curvature_estimator;

  std::cout << "[INFO][MeshSpliter::autoSplitMesh]" << std::endl;
  std::cout << "\t start toMeshTotalCurvature..." << std::endl;
  const Eigen::VectorXd mesh_curvatures =
      curvature_estimator.toMeshTotalCurvature(mesh_ptr);
  if (mesh_curvatures.size() == 0) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMesh]" << std::endl;
    std::cerr << "\t toMeshTotalCurvature failed!" << std::endl;
    std::cerr << "\t mesh_file_path: " << mesh_file_path << std::endl;
    return false;
  }

  // renderMeshCurvature(mesh_ptr, mesh_curvatures);

  std::cout << "[INFO][MeshSpliter::autoSplitMesh]" << std::endl;
  std::cout << "\t start splitMeshByCurvature..." << std::endl;
  const std::unordered_map<int, std::set<int>> sub_mesh_face_idx_set_map =
      splitMeshByCurvature(mesh_ptr, mesh_curvatures, max_merge_curvature,
                           save_painted_mesh_file_path);
  if (sub_mesh_face_idx_set_map.empty()) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMesh]" << std::endl;
    std::cerr << "\t splitMeshByCurvature failed!" << std::endl;
    std::cerr << "\t mesh_file_path: " << mesh_file_path << std::endl;
    return false;
  }

  if (!saveSubMeshes(mesh_ptr, sub_mesh_face_idx_set_map, save_folder_path,
                     overwrite)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMesh]" << std::endl;
    std::cerr << "\t saveSubMeshes failed!" << std::endl;
    std::cerr << "\t mesh_file_path: " << mesh_file_path << std::endl;
    std::cerr << "\t save_folder_path: " << save_folder_path << std::endl;
    return false;
  }

  return true;
}
