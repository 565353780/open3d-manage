#include "Curvature/render.h"

Eigen::Vector3d scalar_to_color(const double &scalar, const double &min_val,
                                const double &max_val) {
  double value = (scalar - min_val) / (max_val - min_val);
  double r = 1.0, g = 1.0, b = 1.0;

  if (value < 0.5) {
    r = value * 2.0;
    g = value * 2.0;
    b = 1.0;
  } else {
    r = 1.0;
    g = 1.0 - (value - 0.5) * 2.0;
    b = 1.0 - (value - 0.5) * 2.0;
  }

  return Eigen::Vector3d(r, g, b);
}

const bool
renderMeshCurvature(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
                    const Eigen::VectorXd &curvatures) {
  const int vertex_num = mesh_ptr->vertices_.size();

  if (vertex_num != curvatures.size()) {
    std::cout << "[ERROR][render::renderMeshCurvature]" << std::endl;
    std::cout << "\t mesh vertices.size() != curvatures.size()!" << std::endl;
    return false;
  }

  // Find the min and max values in scalar_values
  const Eigen::VectorXd k_S_vis = curvatures.array().pow(0.0425);

  const double min_val = k_S_vis.minCoeff();
  const double max_val = k_S_vis.maxCoeff();

  // Convert scalar values into colormap and assign to the vertices
  mesh_ptr->vertex_colors_.resize(vertex_num);
  for (size_t i = 0; i < vertex_num; ++i) {
    mesh_ptr->vertex_colors_[i] = scalar_to_color(k_S_vis(i), min_val, max_val);
  }

  open3d::visualization::DrawGeometries(
      {mesh_ptr}, "[render::renderMeshCurvature] Mesh with Colormap");

  return true;
}

const bool renderPointCloudCurvature(
    std::shared_ptr<open3d::geometry::PointCloud> &pcd_ptr,
    const Eigen::VectorXd &curvatures) {
  const int point_num = pcd_ptr->points_.size();

  // Apply the color map to the point cloud
  // Find the min and max values in scalar_values
  Eigen::VectorXd k_S_PCD_vis = curvatures.array().abs().pow(0.0425);
  double min_val_pcd = k_S_PCD_vis.minCoeff();
  double max_val_pcd = k_S_PCD_vis.maxCoeff();
  pcd_ptr->colors_.resize(point_num);
  for (int i = 0; i < point_num; ++i) {
    pcd_ptr->colors_[i] =
        scalar_to_color(k_S_PCD_vis(i), min_val_pcd, max_val_pcd);
  }

  open3d::visualization::DrawGeometries(
      {pcd_ptr},
      "[render::renderPointCloudCurvature] PointCloud with Colormap");

  return true;
}
