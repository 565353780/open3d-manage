#include "MeshSplit/sub_mesh_manager.h"
#include <algorithm>
#include <filesystem>
#include <random>

SubMeshManager::SubMeshManager(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr) {
  loadMesh(mesh_ptr);
}

const bool SubMeshManager::reset() {
  vertex_set_idx_vec.clear();
  sub_mesh_face_idx_set_map.clear();
  new_sub_set_idx = -1;

  return true;
}

const bool SubMeshManager::loadMesh(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr) {
  reset();
  o3d_mesh_ptr = mesh_ptr;

  mesh = toOpenMesh(mesh_ptr);
  vertex_set_idx_vec = std::vector<int>(mesh.n_vertices(), -1);

  return true;
}

const int SubMeshManager::getVertexSetIdx(const int &vertex_idx) {
  return vertex_set_idx_vec[vertex_idx];
}

const int SubMeshManager::getFreeVertexNum() {
  const int free_vertex_num =
      std::count(vertex_set_idx_vec.begin(), vertex_set_idx_vec.end(), -1);

  return free_vertex_num;
}

const bool SubMeshManager::createNewSubSet() {
  ++new_sub_set_idx;

  sub_mesh_face_idx_set_map[new_sub_set_idx] = std::set<int>();

  return true;
}

const bool SubMeshManager::addVertexIntoNewSubSet(const int &vertex_idx) {
  vertex_set_idx_vec[vertex_idx] = new_sub_set_idx;

  // 将其相邻的面全部归类
  for (TriMesh::VertexFaceIter vf_it =
           mesh.vf_iter(mesh.vertex_handle(vertex_idx));
       vf_it.is_valid(); ++vf_it) {
    sub_mesh_face_idx_set_map[new_sub_set_idx].insert(vf_it->idx());
  }

  return true;
}

const bool SubMeshManager::mergeSubSet(const int &set_idx_1,
                                       const int &set_idx_2) {
  if (set_idx_1 == set_idx_2) {
    return true;
  }

  for (const int &face_idx : sub_mesh_face_idx_set_map[set_idx_2]) {
    sub_mesh_face_idx_set_map[set_idx_1].insert(face_idx);
  }

  // 更新所有点的集合id
  std::replace(vertex_set_idx_vec.begin(), vertex_set_idx_vec.end(), set_idx_2,
               set_idx_1);

  // 删除被合并的集合
  auto it = sub_mesh_face_idx_set_map.find(set_idx_2);
  if (it != sub_mesh_face_idx_set_map.end()) {
    sub_mesh_face_idx_set_map.erase(it);
  }

  return true;
}

const bool SubMeshManager::updateVertexNeighboorInfo(
    const int &vertex_idx, const std::vector<double> &curvatures_vec,
    const float &max_merge_curvature) {
  const int vertex_set_idx = getVertexSetIdx(vertex_idx);

  for (TriMesh::VertexFaceIter vf_it =
           mesh.vf_iter(mesh.vertex_handle(vertex_idx));
       vf_it.is_valid(); ++vf_it) {

    // 将其相邻的面的相邻顶点全部归类
    for (TriMesh::FaceVertexIter fv_it = mesh.fv_iter(*vf_it); fv_it.is_valid();
         ++fv_it) {
      const int neighboor_vertex_idx = fv_it->idx();
      const int neighboor_vertex_set_idx =
          getVertexSetIdx(neighboor_vertex_idx);

      if (neighboor_vertex_set_idx != vertex_set_idx) {
        if (neighboor_vertex_set_idx == -1) {
          vertex_set_idx_vec[neighboor_vertex_idx] = vertex_set_idx;
        }
        // 如果该相邻顶点已经被归类到其他集合，且曲率小于合并阈值
        else if (curvatures_vec[neighboor_vertex_idx] <= max_merge_curvature) {
          mergeSubSet(vertex_set_idx, neighboor_vertex_set_idx);
        }
      }
    }
  }

  return true;
}

const bool
SubMeshManager::addVertexIntoSubSet(const int &vertex_idx,
                                    const std::vector<double> &curvatures_vec,
                                    const float &max_merge_curvature) {
  const int vertex_set_idx = getVertexSetIdx(vertex_idx);

  // 如果当前点还未被归类
  if (vertex_set_idx == -1) {
    // 构建新的子网格集合
    createNewSubSet();

    addVertexIntoNewSubSet(vertex_idx);
  }

  updateVertexNeighboorInfo(vertex_idx, curvatures_vec, max_merge_curvature);

  return true;
}

const bool SubMeshManager::paintSubMesh() {
  const int vertex_num = o3d_mesh_ptr->vertices_.size();

  std::vector<Eigen::Vector3d> random_set_colors;
  random_set_colors.reserve(sub_mesh_face_idx_set_map.size() + 1);

  random_set_colors.emplace_back(Eigen::Vector3d(0.0, 0.0, 0.0));

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  for (int i = 0; i < vertex_num; ++i) {
    random_set_colors.emplace_back(
        Eigen::Vector3d(dist(gen), dist(gen), dist(gen)));
  }

  o3d_mesh_ptr->vertex_colors_.resize(vertex_num);
  for (int i = 0; i < o3d_mesh_ptr->vertices_.size(); ++i) {
    const int vertex_set_idx = vertex_set_idx_vec[i] + 1;

    o3d_mesh_ptr->vertex_colors_[i] = random_set_colors[vertex_set_idx];
  }

  return true;
}

const bool SubMeshManager::renderSubMeshes() {
  if (!paintSubMesh()) {
    std::cerr << "[ERROR][SubMeshManager::renderSubMeshes]" << std::endl;
    std::cerr << "\t paintSubMesh failed!" << std::endl;
    return false;
  }

  open3d::visualization::DrawGeometries(
      {o3d_mesh_ptr}, "[SubMeshManager::renderSubMeshes] sub meshes");

  return true;
}

const bool SubMeshManager::savePaintedMesh(const std::string &save_file_path,
                                           const bool &overwrite) {
  if (std::filesystem::exists(save_file_path)) {
    if (!overwrite) {
      return true;
    }

    std::filesystem::remove(save_file_path);
  }

  const std::string save_folder_path =
      std::filesystem::path(save_file_path).parent_path();
  if (!std::filesystem::exists(save_folder_path)) {
    std::filesystem::create_directories(save_folder_path);
  }

  open3d::io::WriteTriangleMesh(save_file_path, *o3d_mesh_ptr, true);
  return true;
}
