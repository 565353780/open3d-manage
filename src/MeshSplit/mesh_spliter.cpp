#include "MeshSplit/mesh_spliter.h"
#include "Curvature/curvature_estimator.h"
#include <filesystem>
#include <string>

MeshSpliter::MeshSpliter(const std::string &mesh_file_path) {
  sub_mesh_manager_.loadMeshFile(mesh_file_path);
}

const bool MeshSpliter::splitMeshByFaceConnectivity(
    const std::string &save_painted_mesh_file_path) {
  sub_mesh_manager_.toSubMeshesByFaceConnectivity();

  if (save_painted_mesh_file_path != "") {
    const std::string save_painted_mesh_folder_path =
        std::filesystem::path(save_painted_mesh_file_path)
            .parent_path()
            .string();

    if (!std::filesystem::exists(save_painted_mesh_folder_path)) {
      std::filesystem::create_directories(save_painted_mesh_folder_path);
    }

    sub_mesh_manager_.paintSubMesh();
    // sub_mesh_manager_.renderSubMeshes();
    sub_mesh_manager_.savePaintedMesh(save_painted_mesh_file_path, true);
  }

  return true;
}

const bool MeshSpliter::splitMeshByVertexCurvature(
    const Eigen::VectorXd &mesh_curvatures, const float &max_merge_curvature,
    const std::string &save_painted_mesh_file_path) {
  const int vertex_num = sub_mesh_manager_.mesh.n_vertices();

  if (vertex_num != mesh_curvatures.size()) {
    std::cout << "[ERROR][MeshSpliter::splitMeshByVertexCurvature]"
              << std::endl;
    std::cout << "\t mesh vertex num != mesh curvatures num!" << std::endl;
    return false;
  }

  std::vector<double> curvatures_vec(mesh_curvatures.data(),
                                     mesh_curvatures.data() + vertex_num);

  sub_mesh_manager_.toSubMeshesByVertexCurvature(curvatures_vec,
                                                 max_merge_curvature);

  if (save_painted_mesh_file_path != "") {
    const std::string save_painted_mesh_folder_path =
        std::filesystem::path(save_painted_mesh_file_path)
            .parent_path()
            .string();

    if (!std::filesystem::exists(save_painted_mesh_folder_path)) {
      std::filesystem::create_directories(save_painted_mesh_folder_path);
    }

    sub_mesh_manager_.paintSubMesh();
    // sub_mesh_manager_.renderSubMeshes();
    sub_mesh_manager_.savePaintedMesh(save_painted_mesh_file_path, true);
  }

  return true;
}

const bool MeshSpliter::splitMeshByFaceNormal(
    const float &max_merge_angle,
    const std::string &save_painted_mesh_file_path) {
  sub_mesh_manager_.toSubMeshesByFaceNormal(max_merge_angle);

  if (save_painted_mesh_file_path != "") {
    const std::string save_painted_mesh_folder_path =
        std::filesystem::path(save_painted_mesh_file_path)
            .parent_path()
            .string();

    if (!std::filesystem::exists(save_painted_mesh_folder_path)) {
      std::filesystem::create_directories(save_painted_mesh_folder_path);
    }

    sub_mesh_manager_.paintSubMesh();
    // sub_mesh_manager_.renderSubMeshes();
    sub_mesh_manager_.savePaintedMesh(save_painted_mesh_file_path, true);
  }

  return true;
}

std::shared_ptr<open3d::geometry::TriangleMesh>
MeshSpliter::toSubMesh(const std::set<int> &sub_mesh_face_idx_set,
                       const bool &save_colors) {
  const std::vector<Eigen::Vector3d> &vertices =
      sub_mesh_manager_.o3d_mesh_ptr->vertices_;
  const std::vector<Eigen::Vector3i> &triangles =
      sub_mesh_manager_.o3d_mesh_ptr->triangles_;

  std::vector<Eigen::Vector3d> vertex_colors;
  if (save_colors) {
    vertex_colors = sub_mesh_manager_.o3d_mesh_ptr->vertex_colors_;
  }

  std::vector<Eigen::Vector3d> sub_mesh_vertices;
  std::vector<Eigen::Vector3i> sub_mesh_triangles;
  std::vector<Eigen::Vector3d> sub_mesh_vertex_colors;
  std::unordered_map<int, int> vertex_idx_map;

  int new_vertex_index = 0;
  for (int i : sub_mesh_face_idx_set) {
    const Eigen::Vector3i &triangle = triangles[i];
    Eigen::Vector3i new_triangle;

    for (int j = 0; j < 3; ++j) {
      const int &vertex_index = triangle[j];

      if (vertex_idx_map.find(vertex_index) == vertex_idx_map.end()) {
        vertex_idx_map[vertex_index] = new_vertex_index;
        ++new_vertex_index;

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

const bool MeshSpliter::saveSubMeshes(const std::string &save_folder_path,
                                      const bool &need_simplify,
                                      const bool &overwrite) {
  if (std::filesystem::exists(save_folder_path)) {
    if (!overwrite) {
      return true;
    }

    std::filesystem::remove_all(save_folder_path);
  }

  std::filesystem::create_directories(save_folder_path);

  std::unordered_map<int, std::set<int>> sub_mesh_face_idx_set_map;
  if (need_simplify) {
    sub_mesh_face_idx_set_map = toSimplifiedSubMeshFaceIdxSetMap();
  } else {
    sub_mesh_face_idx_set_map = sub_mesh_manager_.sub_mesh_face_idx_set_map;
  }

  for (auto it = sub_mesh_face_idx_set_map.begin();
       it != sub_mesh_face_idx_set_map.end(); ++it) {
    const int &sub_mesh_idx = it->first;
    const std::set<int> &sub_mesh_face_idx_set = it->second;

    std::shared_ptr<open3d::geometry::TriangleMesh> sub_mesh_ptr =
        toSubMesh(sub_mesh_face_idx_set);

    const std::string save_file_path =
        save_folder_path + "sub_mesh_" + std::to_string(sub_mesh_idx) + ".ply";

    open3d::io::WriteTriangleMesh(save_file_path, *sub_mesh_ptr, true);
  }

  return true;
}

const std::unordered_map<int, std::set<int>>
MeshSpliter::toSimplifiedSubMeshFaceIdxSetMap() {
  const int min_sub_mesh_face_num = std::fmax(
      int(sub_mesh_manager_.sub_mesh_face_idx_set_map.at(0).size() / 100.0), 1);

  std::unordered_map<int, std::set<int>> simplified_sub_mesh_face_idx_set_map;

  int new_sub_mesh_idx = 0;
  for (int i = 0; i < sub_mesh_manager_.sub_mesh_face_idx_set_map.size(); ++i) {
    const std::set<int> &sub_mesh_face_idx_set =
        sub_mesh_manager_.sub_mesh_face_idx_set_map.at(i);

    if (sub_mesh_face_idx_set.size() < min_sub_mesh_face_num) {
      continue;
    }

    simplified_sub_mesh_face_idx_set_map[new_sub_mesh_idx] =
        sub_mesh_face_idx_set;

    ++new_sub_mesh_idx;
  }

  return simplified_sub_mesh_face_idx_set_map;
}

const bool MeshSpliter::autoSplitMeshByFaceConnectivity(
    const std::string &save_folder_path,
    const std::string &save_painted_mesh_file_path, const bool &need_simplify,
    const bool &overwrite) {
  if (!overwrite) {
    if (std::filesystem::exists(save_folder_path) &&
        std::filesystem::exists(save_painted_mesh_file_path)) {
      return true;
    }
  }

  std::cout << "[INFO][MeshSpliter::autoSplitMeshByFaceConnectivity]"
            << std::endl;
  std::cout << "\t start splitMeshByFaceConnectivity..." << std::endl;
  if (!splitMeshByFaceConnectivity(save_painted_mesh_file_path)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByFaceConnectivity]"
              << std::endl;
    std::cerr << "\t splitMeshByFaceConnectivity failed!" << std::endl;

    return false;
  }

  if (!saveSubMeshes(save_folder_path, need_simplify, overwrite)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByFaceConnectivity]"
              << std::endl;
    std::cerr << "\t saveSubMeshes failed!" << std::endl;
    std::cerr << "\t save_folder_path: " << save_folder_path << std::endl;
    return false;
  }

  return true;
}

const bool MeshSpliter::autoSplitMeshByVertexCurvature(
    const std::string &save_folder_path, const float &max_merge_curvature,
    const std::string &save_painted_mesh_file_path, const bool &need_simplify,
    const bool &overwrite) {
  if (!overwrite) {
    if (std::filesystem::exists(save_folder_path) &&
        std::filesystem::exists(save_painted_mesh_file_path)) {
      return true;
    }
  }

  CurvatureEstimator curvature_estimator;

  std::cout << "[INFO][MeshSpliter::autoSplitMeshByVertexCurvature]"
            << std::endl;
  std::cout << "\t start toMeshTotalCurvature..." << std::endl;
  const Eigen::VectorXd mesh_curvatures =
      curvature_estimator.toMeshTotalCurvature(sub_mesh_manager_.o3d_mesh_ptr);
  if (mesh_curvatures.size() == 0) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByVertexCurvature]"
              << std::endl;
    std::cerr << "\t toMeshTotalCurvature failed!" << std::endl;
    return false;
  }

  // TODO: make this controllable later
  const float min_value = float(mesh_curvatures.minCoeff());

  // renderMeshCurvature(mesh_ptr, mesh_curvatures);

  std::cout << "[INFO][MeshSpliter::autoSplitMeshByVertexCurvature]"
            << std::endl;
  std::cout << "\t start splitMeshByVertexCurvature..." << std::endl;
  if (!splitMeshByVertexCurvature(mesh_curvatures,
                                  max_merge_curvature * min_value,
                                  save_painted_mesh_file_path)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByVertexCurvature]"
              << std::endl;
    std::cerr << "\t splitMeshByVertexCurvature failed!" << std::endl;
    return false;
  }

  if (!saveSubMeshes(save_folder_path, need_simplify, overwrite)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByVertexCurvature]"
              << std::endl;
    std::cerr << "\t saveSubMeshes failed!" << std::endl;
    std::cerr << "\t save_folder_path: " << save_folder_path << std::endl;
    return false;
  }

  return true;
}

const bool MeshSpliter::autoSplitMeshByFaceNormal(
    const std::string &save_folder_path, const float &max_merge_angle,
    const std::string &save_painted_mesh_file_path, const bool &need_simplify,
    const bool &overwrite) {
  if (!overwrite) {
    if (std::filesystem::exists(save_folder_path) &&
        std::filesystem::exists(save_painted_mesh_file_path)) {
      return true;
    }
  }

  std::cout << "[INFO][MeshSpliter::autoSplitMeshByFaceNormal]" << std::endl;
  std::cout << "\t start splitMeshByFaceNormal..." << std::endl;
  if (!splitMeshByFaceNormal(max_merge_angle, save_painted_mesh_file_path)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByFaceNormal]" << std::endl;
    std::cerr << "\t splitMeshByVertexCurvature failed!" << std::endl;
    return false;
  }

  if (!saveSubMeshes(save_folder_path, need_simplify, overwrite)) {
    std::cerr << "[ERROR][MeshSpliter::autoSplitMeshByFaceNormal]" << std::endl;
    std::cerr << "\t saveSubMeshes failed!" << std::endl;
    std::cerr << "\t save_folder_path: " << save_folder_path << std::endl;
    return false;
  }

  return true;
}
