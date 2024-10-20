#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <open3d/Open3D.h>
#include <string>

class CurvatureEstimator {
public:
  CurvatureEstimator() {};

  std::shared_ptr<open3d::geometry::TriangleMesh>
  toMeshTotalCurvature(const std::string &mesh_file_path);

  std::shared_ptr<open3d::geometry::PointCloud>
  toPcdTotalCurvature(const std::string &pcd_file_path);

  // private:
};
