#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>

class MeshSpliter {
public:
  MeshSpliter() {};

  const bool splitMeshByCurvature(
      std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
      const Eigen::VectorXd &mesh_curvatures,
      const float &max_merge_curvature = 10.0);
};
