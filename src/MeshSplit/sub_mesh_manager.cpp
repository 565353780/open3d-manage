#include "MeshSplit/sub_mesh_manager.h"

SubMeshManager::SubMeshManager(TriMesh &mesh_value) {
  loadOpenMesh(mesh_value);
}

const bool SubMeshManager::reset() {
  vertex_set_idx_vec.clear();
  sub_mesh_face_idx_set_map.clear();
  new_sub_set_idx = -1;

  return true;
}

const bool SubMeshManager::loadOpenMesh(TriMesh &mesh_value) {
  reset();

  mesh = mesh_value;
  vertex_set_idx_vec = std::vector<int>(mesh.n_vertices(), -1);

  return true;
}

const int SubMeshManager::getVertexSetIdx(const int &vertex_idx) {
  return vertex_set_idx_vec[vertex_idx];
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
