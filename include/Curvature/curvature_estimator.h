#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <memory>
#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>

class CurvatureEstimator {
public:
  CurvatureEstimator() {};

  const Eigen::VectorXd toMeshTotalCurvature(
      std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr);

  const Eigen::VectorXd
  toPcdTotalCurvature(std::shared_ptr<open3d::geometry::PointCloud> &pcd_ptr);

  const Eigen::VectorXd
  toMeshFileTotalCurvature(const std::string &mesh_file_path);

  const Eigen::VectorXd
  toPcdFileTotalCurvature(const std::string &pcd_file_path);
};
