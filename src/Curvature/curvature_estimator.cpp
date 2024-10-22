#include "Curvature/curvature_estimator.h"
#include "Curvature/TotalCurvature.h"
#include "Curvature/TotalCurvaturePointCloud.h"
#include "Curvature/io.h"
#include <iostream>
#include <memory>
#include <open3d/geometry/PointCloud.h>

const Eigen::VectorXd CurvatureEstimator::toMeshTotalCurvature(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr) {
  Eigen::VectorXd k_S;

  if (mesh_ptr->IsEmpty()) {
    std::cout << "[ERROR][CurvatureEstimator::toMeshTotalCurvature]"
              << std::endl;
    std::cout << "\t mesh ptr is empty!" << std::endl;

    return k_S;
  }

  mesh_ptr->ComputeVertexNormals();

  Eigen::MatrixXd V, N;
  Eigen::MatrixXi F;
  V = Eigen::Map<
      const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      reinterpret_cast<const double *>(mesh_ptr->vertices_.data()),
      mesh_ptr->vertices_.size(), 3);
  F = Eigen::Map<const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      reinterpret_cast<const int *>(mesh_ptr->triangles_.data()),
      mesh_ptr->triangles_.size(), 3);
  N = Eigen::Map<
      const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>(
      reinterpret_cast<const double *>(mesh_ptr->vertex_normals_.data()),
      mesh_ptr->vertex_normals_.size(), 3);

  // calculate total curvature on triangle mesh
  open3d::geometry::TotalCurvature::TotalCurvatureMesh(V, F, N, k_S);

  return k_S;
}

const Eigen::VectorXd CurvatureEstimator::toPcdTotalCurvature(
    std::shared_ptr<open3d::geometry::PointCloud> &pcd_ptr) {
  Eigen::VectorXd k_S_PCD;

  if (pcd_ptr->IsEmpty()) {
    std::cout << "[ERROR][CurvatureEstimator::toPcdTotalCurvature]"
              << std::endl;
    std::cout << "\t pcd ptr is empty!" << std::endl;

    return k_S_PCD;
  }

  if (!pcd_ptr->HasNormals()) {
    pcd_ptr->EstimateNormals();
  }

  std::vector<Eigen::Vector3d> points_v = pcd_ptr->points_;
  std::vector<Eigen::Vector3d> points_n = pcd_ptr->normals_;
  Eigen::MatrixXd V_PCD(points_v.size(), 3);
  Eigen::MatrixXd N_PCD(points_v.size(), 3);
  for (size_t i = 0; i < points_v.size(); ++i) {
    V_PCD.row(i) = points_v[i];
    N_PCD.row(i) = points_n[i];
  }

  k_S_PCD.resize(V_PCD.rows());

  open3d::geometry::TotalCurvaturePointCloud::TotalCurvaturePCD(V_PCD, N_PCD,
                                                                k_S_PCD, 20);

  return k_S_PCD;
}

const Eigen::VectorXd CurvatureEstimator::toMeshFileTotalCurvature(
    const std::string &mesh_file_path) {
  Eigen::VectorXd mesh_total_curvature;

  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
      loadMeshFile(mesh_file_path);

  if (mesh_ptr->IsEmpty()) {
    std::cout << "[ERROR][CurvatureEstimator::toMeshFileTotalCurvature]"
              << std::endl;
    std::cout << "\t loadMeshFile failed!" << std::endl;

    return mesh_total_curvature;
  }

  mesh_total_curvature = toMeshTotalCurvature(mesh_ptr);

  return mesh_total_curvature;
}

const Eigen::VectorXd
CurvatureEstimator::toPcdFileTotalCurvature(const std::string &pcd_file_path) {
  Eigen::VectorXd pcd_total_curvature;

  std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr =
      loadPcdFile(pcd_file_path);

  if (pcd_ptr->IsEmpty()) {
    std::cout << "[ERROR][CurvatureEstimator::toPcdFileTotalCurvature]"
              << std::endl;
    std::cout << "\t loadPcdFile failed!" << std::endl;

    return pcd_total_curvature;
  }

  pcd_total_curvature = toPcdTotalCurvature(pcd_ptr);

  return pcd_total_curvature;
}
