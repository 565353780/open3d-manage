#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <open3d/Open3D.h>

typedef OpenMesh::TriMesh_ArrayKernelT<> TriMesh;

TriMesh toOpenMesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr);
