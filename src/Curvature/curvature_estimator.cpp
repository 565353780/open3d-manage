#include "Curvature/curvature_estimator.h"
#include "Curvature/TotalCurvature.h"
#include "Curvature/TotalCurvaturePointCloud.h"
#include "Curvature/render.h"
#include <filesystem>
#include <iostream>
#include <memory>
#include <open3d/geometry/PointCloud.h>

std::shared_ptr<open3d::geometry::TriangleMesh>
CurvatureEstimator::toMeshTotalCurvature(const std::string &mesh_file_path) {
  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
      std::make_shared<open3d::geometry::TriangleMesh>();

  if (!std::filesystem::exists(mesh_file_path)) {
    std::cout << "[ERROR][CurvatureEstimator::toMeshTotalCurvature]"
              << std::endl;
    std::cout << "\t mesh file not exist!" << std::endl;
    std::cout << "\t mesh_file_path: " << mesh_file_path << std::endl;
    return mesh_ptr;
  }

  if (!open3d::io::ReadTriangleMesh(mesh_file_path, *mesh_ptr)) {
    std::cout << "[ERROR][CurvatureEstimator::toMeshTotalCurvature]"
              << std::endl;
    std::cout << "\t ReadTriangleMesh failed!" << std::endl;
    std::cout << "\t mesh_file_path: " << mesh_file_path << std::endl;
    mesh_ptr.reset();
    return mesh_ptr;
  }

  mesh_ptr->ComputeVertexNormals();

  Eigen::MatrixXd V, N;
  Eigen::MatrixXi F;
  Eigen::VectorXd k_S;
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

  // Find the min and max values in scalar_values
  Eigen::VectorXd k_S_vis = k_S.array().pow(0.0425);
  double min_val = k_S_vis.minCoeff();
  double max_val = k_S_vis.maxCoeff();

  // Convert scalar values into colormap and assign to the vertices
  mesh_ptr->vertex_colors_.resize(mesh_ptr->vertices_.size());
  for (size_t i = 0; i < mesh_ptr->vertices_.size(); ++i) {
    mesh_ptr->vertex_colors_[i] = scalar_to_color(k_S_vis(i), min_val, max_val);
  }

  open3d::visualization::DrawGeometries({mesh_ptr}, "Mesh with Colormap");

  return mesh_ptr;
}

std::shared_ptr<open3d::geometry::PointCloud>
CurvatureEstimator::toPcdTotalCurvature(const std::string &pcd_file_path) {
  std::shared_ptr<open3d::geometry::PointCloud> point_cloud_ptr =
      std::make_shared<open3d::geometry::PointCloud>();

  if (!std::filesystem::exists(pcd_file_path)) {
    std::cout << "[ERROR][CurvatureEstimator::toPcdTotalCurvature]"
              << std::endl;
    std::cout << "\t pcd file not exist!" << std::endl;
    std::cout << "\t pcd_file_path: " << pcd_file_path << std::endl;
    return point_cloud_ptr;
  }

  open3d::io::ReadPointCloud(pcd_file_path, *point_cloud_ptr);

  if (!point_cloud_ptr->HasNormals()) {
    point_cloud_ptr->EstimateNormals();
  }

  std::vector<Eigen::Vector3d> points_v = point_cloud_ptr->points_;
  std::vector<Eigen::Vector3d> points_n = point_cloud_ptr->normals_;
  Eigen::MatrixXd V_PCD(points_v.size(), 3);
  Eigen::MatrixXd N_PCD(points_v.size(), 3);
  for (size_t i = 0; i < points_v.size(); ++i) {
    V_PCD.row(i) = points_v[i];
    N_PCD.row(i) = points_n[i];
  }

  Eigen::VectorXd k_S_PCD(V_PCD.rows());

  open3d::geometry::TotalCurvaturePointCloud::TotalCurvaturePCD(V_PCD, N_PCD,
                                                                k_S_PCD, 20);

  // Apply the color map to the point cloud
  // Find the min and max values in scalar_values
  Eigen::VectorXd k_S_PCD_vis = k_S_PCD.array().abs().pow(0.0425);
  double min_val_pcd = k_S_PCD_vis.minCoeff();
  double max_val_pcd = k_S_PCD_vis.maxCoeff();
  point_cloud_ptr->colors_.resize(point_cloud_ptr->points_.size());
  for (int i = 0; i < point_cloud_ptr->points_.size(); ++i) {
    point_cloud_ptr->colors_[i] =
        scalar_to_color(k_S_PCD_vis(i), min_val_pcd, max_val_pcd);
  }

  open3d::visualization::DrawGeometries({point_cloud_ptr},
                                        "PointCloud with Colormap");

  return point_cloud_ptr;
}
