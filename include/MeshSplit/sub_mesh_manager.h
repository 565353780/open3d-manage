#pragma once
#include "MeshSplit/trans.h"
#include <open3d/Open3D.h>

class SubMeshManager {
public:
  SubMeshManager() {};

  SubMeshManager(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr);

  SubMeshManager(const std::string &mesh_file_path);

  const bool reset();

  const bool
  loadMesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr);

  const bool loadMeshFile(const std::string &mesh_file_path);

  const float toFaceNormalAngle(const int &face_idx_1, const int &face_idx_2);

  const int getVertexSetIdx(const int &vertex_idx);

  const int getFaceSetIdx(const int &face_idx);

  const int getFreeVertexNum();

  const int getFreeFaceNum();

  const bool createNewSubSet();

  const bool setSubMeshIdxForFaceVertices(const int &face_idx);

  const bool setSubMeshIdxForNeighboorFacesAndVertices(const int &vertex_idx);

  const bool addVertexIntoNewSubSet(const int &vertex_idx);

  const bool addFaceIntoNewSubSet(const int &face_idx);

  const bool mergeSubSet(const int &set_idx_1, const int &set_idx_2);

  const bool updateConflictFaceSetIdx(const int &face_idx,
                                      const std::vector<double> &curvatures_vec,
                                      const float &max_merge_curvature);

  const bool
  updateVertexNeighboorInfo(const int &vertex_idx,
                            const std::vector<double> &curvatures_vec,
                            const float &max_merge_curvature);

  const bool updateFaceSetIdx(const int &face_idx,
                              const float &max_merge_angle);

  const bool updateFaceNeighboorInfo(const int &face_idx,
                                     const float &max_merge_angle);

  const bool addVertexIntoSubSet(const int &vertex_idx,
                                 const std::vector<double> &curvatures_vec,
                                 const float &max_merge_curvature);

  const bool
  addDistanceFaceIntoSubSet(const int &face_idx,
                            const std::vector<float> &vertex_pcd_distance_vec,
                            const std::vector<float> &max_distance_level_vec);

  const bool addConnectedFaceIntoSubSet(const int &face_idx);

  const bool addFaceIntoSubSet(const int &face_idx,
                               const float &max_merge_angle);

  const bool sortSubMeshIdxSetMap();

  const std::vector<int> toUnusedFaceIdxVec();

  const bool checkSubMeshState();

  // outer function algo 1
  const bool toSubMeshesByFacePcdDistance(
      const std::string &pcd_file_path,
      const std::vector<float> &max_distance_level_vec);

  // outer function algo 1
  const bool toSubMeshesByFaceConnectivity();

  // outer function algo 2
  const bool
  toSubMeshesByVertexCurvature(const std::vector<double> &curvatures_vec,
                               const float &max_merge_curvature);

  // outer function algo 3
  const bool toSubMeshesByFaceNormal(const float &max_merge_angle);

  const bool paintFaceVertices(const int &face_idx,
                               const Eigen::Vector3d &color);

  const bool paintSubMesh();
  const bool renderSubMeshes();
  const bool savePaintedMesh(const std::string &save_file_path,
                             const bool &overwrite = false);

public:
  std::shared_ptr<open3d::geometry::TriangleMesh> o3d_mesh_ptr;
  std::shared_ptr<open3d::geometry::TriangleMesh> face_paint_o3d_mesh_ptr;
  TriMesh mesh;

  std::vector<int> vertex_set_idx_vec;
  std::vector<int> face_set_idx_vec;
  std::unordered_map<int, std::set<int>> sub_mesh_face_idx_set_map;
  // 自增idx，确保map的key不重复
  int new_sub_set_idx = -1;
};
