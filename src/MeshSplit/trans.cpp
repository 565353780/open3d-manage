#include "MeshSplit/trans.h"

TriMesh toOpenMesh(std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr) {
  mesh_ptr->OrientTriangles();

  TriMesh mesh;

  for (int i = 0; i < mesh_ptr->vertices_.size(); ++i) {
    const Eigen::Vector3d &vertex = mesh_ptr->vertices_[i];

    mesh.add_vertex(TriMesh::Point(vertex(0), vertex(1), vertex(2)));
  }

  for (int i = 0; i < mesh_ptr->triangles_.size(); ++i) {
    const Eigen::Vector3i &triangle = mesh_ptr->triangles_[i];

    std::vector<TriMesh::VertexHandle> face_vhandles;
    face_vhandles.reserve(3);
    face_vhandles.emplace_back(mesh.vertex_handle(triangle(0)));
    face_vhandles.emplace_back(mesh.vertex_handle(triangle(1)));
    face_vhandles.emplace_back(mesh.vertex_handle(triangle(2)));

    mesh.add_face(face_vhandles);
  }

  return mesh;
}
