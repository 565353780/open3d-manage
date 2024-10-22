#pragma once

#include <open3d/Open3D.h>
#include <string>

std::shared_ptr<open3d::geometry::TriangleMesh>
loadMeshFile(const std::string &mesh_file_path);

std::shared_ptr<open3d::geometry::PointCloud>
loadPcdFile(const std::string &pcd_file_path);
