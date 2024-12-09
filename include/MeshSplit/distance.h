#pragma once

#include <open3d/Open3D.h>

const bool toMeshVertexPcdDistanceVec(
    std::shared_ptr<open3d::geometry::TriangleMesh> &o3d_mesh_ptr,
    const std::string &pcd_file_path,
    std::vector<float> &vertex_pcd_distance_vec);
