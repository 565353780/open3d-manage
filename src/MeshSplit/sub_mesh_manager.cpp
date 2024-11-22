#include "MeshSplit/sub_mesh_manager.h"
#include <algorithm>
#include <filesystem>
#include <memory>
#include <random>
#include <unordered_map>

SubMeshManager::SubMeshManager(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr) {
  loadMesh(mesh_ptr);
}

const bool SubMeshManager::reset() {
  vertex_set_idx_vec.clear();
  face_set_idx_vec.clear();
  sub_mesh_face_idx_set_map.clear();
  new_sub_set_idx = -1;

  return true;
}

const bool SubMeshManager::loadMesh(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr) {
  reset();
  o3d_mesh_ptr = mesh_ptr;
  face_paint_o3d_mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>();

  mesh = toOpenMesh(mesh_ptr);
  vertex_set_idx_vec = std::vector<int>(mesh.n_vertices(), -1);
  face_set_idx_vec = std::vector<int>(mesh.n_faces(), -1);

  return true;
}

const int SubMeshManager::getVertexSetIdx(const int &vertex_idx) {
  return vertex_set_idx_vec[vertex_idx];
}

const int SubMeshManager::getFaceSetIdx(const int &face_idx) {
  return face_set_idx_vec[face_idx];
}

const int SubMeshManager::getFreeVertexNum() {
  const int free_vertex_num =
      std::count(vertex_set_idx_vec.begin(), vertex_set_idx_vec.end(), -1);

  return free_vertex_num;
}

const int SubMeshManager::getFreeFaceNum() {
  const int free_face_num =
      std::count(face_set_idx_vec.begin(), face_set_idx_vec.end(), -1);

  return free_face_num;
}

const bool SubMeshManager::createNewSubSet() {
  ++new_sub_set_idx;

  sub_mesh_face_idx_set_map[new_sub_set_idx] = std::set<int>();

  return true;
}

const bool SubMeshManager::setSubMeshIdxForFaceVertices(const int &face_idx) {
  const int face_set_idx = getFaceSetIdx(face_idx);
  if (face_set_idx == -1) {
    return true;
  }

  for (TriMesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(face_idx));
       fv_it.is_valid(); ++fv_it) {
    const int neighboor_vertex_idx = fv_it->idx();

    if (getVertexSetIdx(neighboor_vertex_idx) != -1) {
      continue;
    }

    vertex_set_idx_vec[neighboor_vertex_idx] = face_set_idx;
  }

  return true;
}

const bool SubMeshManager::setSubMeshIdxForNeighboorFacesAndVertices(
    const int &vertex_idx) {
  const int vertex_set_idx = getVertexSetIdx(vertex_idx);
  if (vertex_set_idx == -1) {
    return true;
  }

  for (TriMesh::VertexFaceIter vf_it =
           mesh.vf_iter(mesh.vertex_handle(vertex_idx));
       vf_it.is_valid(); ++vf_it) {
    const int neighboor_face_idx = vf_it->idx();

    if (getFaceSetIdx(neighboor_face_idx) != -1) {
      continue;
    }

    face_set_idx_vec[neighboor_face_idx] = vertex_set_idx;
    sub_mesh_face_idx_set_map[vertex_set_idx].insert(neighboor_face_idx);

    setSubMeshIdxForFaceVertices(neighboor_face_idx);
  }

  return true;
}

const bool SubMeshManager::addVertexIntoNewSubSet(const int &vertex_idx) {
  createNewSubSet();

  vertex_set_idx_vec[vertex_idx] = new_sub_set_idx;

  return true;
}

const bool SubMeshManager::mergeSubSet(const int &set_idx_1,
                                       const int &set_idx_2) {
  if (set_idx_1 == set_idx_2) {
    return true;
  }

  for (const int face_idx : sub_mesh_face_idx_set_map[set_idx_2]) {
    sub_mesh_face_idx_set_map[set_idx_1].insert(face_idx);
  }

  // 更新所有集合id
  std::replace(vertex_set_idx_vec.begin(), vertex_set_idx_vec.end(), set_idx_2,
               set_idx_1);
  std::replace(face_set_idx_vec.begin(), face_set_idx_vec.end(), set_idx_2,
               set_idx_1);

  // 删除被合并的集合
  sub_mesh_face_idx_set_map.erase(sub_mesh_face_idx_set_map.find(set_idx_2));

  return true;
}

const bool SubMeshManager::updateConflictFaceSetIdx(
    const int &face_idx, const std::vector<double> &curvatures_vec,
    const float &max_merge_curvature) {
  if (getFaceSetIdx(face_idx) != -1) {
    return true;
  }

  std::vector<int> neighboor_vertex_set_idx_vec;
  std::vector<float> neighboor_vertex_curvature_vec;

  for (TriMesh::FaceVertexIter fv_it = mesh.fv_iter(mesh.face_handle(face_idx));
       fv_it.is_valid(); ++fv_it) {
    const int neighboor_vertex_idx = fv_it->idx();

    const int neighboor_vertex_set_idx = getVertexSetIdx(neighboor_vertex_idx);
    if (neighboor_vertex_set_idx == -1) {
      return true;
    }

    const float neighboor_vertex_curvature =
        curvatures_vec[neighboor_vertex_idx];

    neighboor_vertex_set_idx_vec.push_back(neighboor_vertex_set_idx);
    neighboor_vertex_curvature_vec.push_back(neighboor_vertex_curvature);
  }

  auto min_iter = std::min_element(neighboor_vertex_curvature_vec.begin(),
                                   neighboor_vertex_curvature_vec.end());

  const size_t min_index =
      std::distance(neighboor_vertex_curvature_vec.begin(), min_iter);

  if (*min_iter <= max_merge_curvature) {
    const int merge_set_idx = neighboor_vertex_set_idx_vec[min_index];

    face_set_idx_vec[face_idx] = merge_set_idx;
    sub_mesh_face_idx_set_map[merge_set_idx].insert(face_idx);
  } else {
    return true;
    std::cout << neighboor_vertex_set_idx_vec[0] << ", ";
    std::cout << neighboor_vertex_set_idx_vec[1] << ", ";
    std::cout << neighboor_vertex_set_idx_vec[2] << " --> ";
    std::cout << neighboor_vertex_curvature_vec[0] << ", ";
    std::cout << neighboor_vertex_curvature_vec[1] << ", ";
    std::cout << neighboor_vertex_curvature_vec[2] << " > "
              << max_merge_curvature << std::endl;
  }

  return true;
}

const bool SubMeshManager::updateVertexNeighboorInfo(
    const int &vertex_idx, const std::vector<double> &curvatures_vec,
    const float &max_merge_curvature) {
  setSubMeshIdxForNeighboorFacesAndVertices(vertex_idx);

  const int vertex_set_idx = getVertexSetIdx(vertex_idx);

  for (TriMesh::VertexFaceIter vf_it =
           mesh.vf_iter(mesh.vertex_handle(vertex_idx));
       vf_it.is_valid(); ++vf_it) {

    for (TriMesh::FaceVertexIter fv_it = mesh.fv_iter(*vf_it); fv_it.is_valid();
         ++fv_it) {
      const int neighboor_vertex_idx = fv_it->idx();

      const int neighboor_vertex_set_idx =
          getVertexSetIdx(neighboor_vertex_idx);

      if (neighboor_vertex_set_idx != vertex_set_idx) {
        if (curvatures_vec[neighboor_vertex_idx] <= max_merge_curvature) {
          mergeSubSet(vertex_set_idx, neighboor_vertex_set_idx);
        }
      }

      for (TriMesh::VertexFaceIter fvf_it = mesh.vf_iter(*fv_it);
           fvf_it.is_valid(); ++fvf_it) {
        updateConflictFaceSetIdx(fvf_it->idx(), curvatures_vec,
                                 max_merge_curvature);
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

  if (vertex_set_idx == -1) {
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
  if (face_paint_o3d_mesh_ptr->vertex_colors_.size() !=
      face_paint_o3d_mesh_ptr->vertices_.size()) {
    face_paint_o3d_mesh_ptr->vertex_colors_.resize(
        face_paint_o3d_mesh_ptr->vertices_.size(),
        Eigen::Vector3d(0.0, 0.0, 0.0));
  }

  for (int i = 0; i < 3; ++i) {
    face_paint_o3d_mesh_ptr->vertex_colors_[3 * face_idx + i] = color;
  }

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
  const std::vector<Eigen::Vector3d> v = o3d_mesh_ptr->vertices_;
  const std::vector<Eigen::Vector3i> f = o3d_mesh_ptr->triangles_;

  std::vector<Eigen::Vector3d> new_v;
  std::vector<Eigen::Vector3i> new_f;
  new_v.reserve(3 * f.size());
  new_f.reserve(f.size());

  for (int i = 0; i < f.size(); ++i) {
    const Eigen::Vector3i &triangle = f[i];

    for (int j = 0; j < 3; ++j) {
      new_v.push_back(v[triangle[j]]);
    }

    const Eigen::Vector3i new_triangle({3 * i, 3 * i + 1, 3 * i + 2});
    new_f.push_back(new_triangle);
  }

  face_paint_o3d_mesh_ptr->vertices_ = new_v;
  face_paint_o3d_mesh_ptr->triangles_ = new_f;

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
      {face_paint_o3d_mesh_ptr},
      "[SubMeshManager::renderSubMeshes] sub meshes");

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

  open3d::io::WriteTriangleMesh(save_file_path, *face_paint_o3d_mesh_ptr, true);
  return true;
}
