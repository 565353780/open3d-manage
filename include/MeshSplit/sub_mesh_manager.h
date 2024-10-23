#pragma once
#include "MeshSplit/trans.h"

class SubMeshManager {
public:
  SubMeshManager() {};

  SubMeshManager(TriMesh &mesh_value);

  const bool reset();

  const bool loadOpenMesh(TriMesh &mesh_value);

  const int getVertexSetIdx(const int &vertex_idx);

  const bool createNewSubSet();

  const bool addVertexIntoNewSubSet(const int &vertex_idx);

  const bool mergeSubSet(const int &set_idx_1, const int &set_idx_2);

  const bool
  updateVertexNeighboorInfo(const int &vertex_idx,
                            const std::vector<double> &curvatures_vec,
                            const float &max_merge_curvature);

  const bool addVertexIntoSubSet(const int &vertex_idx,
                                 const std::vector<double> &curvatures_vec,
                                 const float &max_merge_curvature);

public:
  TriMesh mesh;

  std::vector<int> vertex_set_idx_vec;
  std::unordered_map<int, std::set<int>> sub_mesh_face_idx_set_map;
  // 自增idx，确保map的key不重复
  int new_sub_set_idx = -1;
};
