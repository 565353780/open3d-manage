#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <open3d/Open3D.h>
#include <string>

class CurvatureEstimator {
public:
  CurvatureEstimator() {};

  const bool toMeshTotalCurvature(const std::string &mesh_file_path);

  const bool toPcdTotalCurvature(const std::string &pcd_file_path);

  // private:
};
