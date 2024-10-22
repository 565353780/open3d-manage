#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>

// Function to convert scalar values into a colormap (Jet colormap in this case)
Eigen::Vector3d scalar_to_color(const double &scalar, const double &min_val,
                                const double &max_val);

const bool
renderMeshCurvature(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
                    const Eigen::VectorXd &curvatures);

const bool renderPointCloudCurvature(
    std::shared_ptr<open3d::geometry::PointCloud> &pcd_ptr,
    const Eigen::VectorXd &curvatures);
