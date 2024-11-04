#include "MeshSplit/sub_mesh_manager.h"
#include <algorithm>
#include <filesystem>
#include <random>
#include <unordered_map>

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

const bool
SubMeshManager::setSubMeshIdxForNeighboorFaces(const int &vertex_idx) {
  const int vertex_set_idx = vertex_set_idx_vec[vertex_idx];
  if (vertex_set_idx == -1) {
    return true;
  }

  for (TriMesh::VertexFaceIter vf_it =
           mesh.vf_iter(mesh.vertex_handle(vertex_idx));
       vf_it.is_valid(); ++vf_it) {
    sub_mesh_face_idx_set_map[vertex_set_idx].insert(vf_it->idx());
  }

  return true;
}

const bool SubMeshManager::addVertexIntoNewSubSet(const int &vertex_idx) {
  vertex_set_idx_vec[vertex_idx] = new_sub_set_idx;

  setSubMeshIdxForNeighboorFaces(vertex_idx);

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

      setSubMeshIdxForNeighboorFaces(neighboor_vertex_idx);

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

const bool SubMeshManager::sortSubMeshIdxSetMap() {
  std::vector<std::pair<int, int>> sub_mesh_idx_size_pair_vec;
  sub_mesh_idx_size_pair_vec.reserve(sub_mesh_face_idx_set_map.size());
  for (auto it = sub_mesh_face_idx_set_map.begin();
       it != sub_mesh_face_idx_set_map.end(); ++it) {
    const int sub_mesh_idx = it->first;
    const std::set<int> sub_mesh_face_idx_set = it->second;

    sub_mesh_idx_size_pair_vec.emplace_back(
        std::pair<int, int>(sub_mesh_idx, sub_mesh_face_idx_set.size()));
  }

  std::sort(sub_mesh_idx_size_pair_vec.begin(),
            sub_mesh_idx_size_pair_vec.end(),
            [](std::pair<int, int> a, std::pair<int, int> b) {
              return a.second > b.second;
            });

  std::unordered_map<int, std::set<int>> new_sub_mesh_face_idx_set_map;
  for (int i = 0; i < sub_mesh_idx_size_pair_vec.size(); ++i) {
    new_sub_mesh_face_idx_set_map[i] =
        sub_mesh_face_idx_set_map[sub_mesh_idx_size_pair_vec[i].first];
  }

  sub_mesh_face_idx_set_map = new_sub_mesh_face_idx_set_map;

  return true;
}

const std::vector<int> SubMeshManager::toUnusedFaceIdxVec() {
  const int face_num = o3d_mesh_ptr->triangles_.size();

  std::vector<bool> face_used_vec(face_num, false);
  for (auto it = sub_mesh_face_idx_set_map.begin();
       it != sub_mesh_face_idx_set_map.end(); ++it) {
    const int sub_mesh_idx = it->first;
    const std::set<int> sub_mesh_face_idx_set = it->second;

    for (int i : sub_mesh_face_idx_set) {
      face_used_vec[i] = true;
    }
  }

  std::vector<int> unused_face_idx_vec;
  for (int i = 0; i < face_num; ++i) {
    if (!face_used_vec[i]) {
      unused_face_idx_vec.emplace_back(i);
    }
  }

  return unused_face_idx_vec;
}

const bool SubMeshManager::checkSubMeshState() {
  std::cout << sub_mesh_face_idx_set_map.size() << std::endl;

  std::cout << "CHECK vertex_set_idx_vec -1 num = "
            << std::count(vertex_set_idx_vec.begin(), vertex_set_idx_vec.end(),
                          -1)
            << std::endl;

  const std::vector<int> unused_face_idx_vec = toUnusedFaceIdxVec();
  std::cout << "CHECK unused face idx num = " << unused_face_idx_vec.size()
            << " / " << o3d_mesh_ptr->triangles_.size() << std::endl;

  std::cout << "CHECK sub set face num = " << std::endl;
  for (auto it = sub_mesh_face_idx_set_map.begin();
       it != sub_mesh_face_idx_set_map.end(); ++it) {
    const int sub_mesh_idx = it->first;
    const std::set<int> sub_mesh_face_idx_set = it->second;
    std::cout << sub_mesh_idx << " -> " << sub_mesh_face_idx_set.size()
              << std::endl;
  }

  return true;
}

const bool SubMeshManager::paintFaceVertices(const int &face_idx,
                                             const Eigen::Vector3d &color) {
  if (o3d_mesh_ptr->vertex_colors_.size() != o3d_mesh_ptr->vertices_.size()) {
    o3d_mesh_ptr->vertex_colors_.resize(o3d_mesh_ptr->vertices_.size(),
                                        Eigen::Vector3d(0.0, 0.0, 0.0));
  }

  for (TriMesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(face_idx));
       fv_it.is_valid(); ++fv_it) {
    const int neighboor_vertex_idx = fv_it->idx();

    o3d_mesh_ptr->vertex_colors_[neighboor_vertex_idx] = color;
  }

  return true;
}

const bool SubMeshManager::paintSubMesh() {
  const int face_num = o3d_mesh_ptr->triangles_.size();

  std::vector<Eigen::Vector3d> random_set_colors;
  random_set_colors.reserve(sub_mesh_face_idx_set_map.size());

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  for (int i = 0; i < sub_mesh_face_idx_set_map.size(); ++i) {
    random_set_colors.emplace_back(
        Eigen::Vector3d(dist(gen), dist(gen), dist(gen)));
  }

  o3d_mesh_ptr->vertex_colors_.resize(o3d_mesh_ptr->vertices_.size(),
                                      Eigen::Vector3d(0.0, 0.0, 0.0));
  int current_paint_sub_mesh_idx = 0;
  for (auto it = sub_mesh_face_idx_set_map.begin();
       it != sub_mesh_face_idx_set_map.end(); ++it) {
    const int sub_mesh_idx = it->first;
    const std::set<int> sub_mesh_face_idx_set = it->second;

    for (int i : sub_mesh_face_idx_set) {
      paintFaceVertices(i, random_set_colors[current_paint_sub_mesh_idx]);
    }

    ++current_paint_sub_mesh_idx;
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
