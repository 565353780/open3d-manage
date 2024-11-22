#pragma once

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <open3d/Open3D.h>

struct MyTraits : public OpenMesh::DefaultTraits {
  FaceAttributes(OpenMesh::Attributes::Normal);
};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> TriMesh;

TriMesh toOpenMesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr);
