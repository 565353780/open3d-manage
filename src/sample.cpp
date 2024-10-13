#include "sample.h"
#include <filesystem>
#include <open3d/Open3D.h>
#include <open3d/io/TriangleMeshIO.h>
#include <unordered_map>

const bool sampleLocalSurfaceNearVertex(const std::string &mesh_file_path,
    const int &vertex_idx, const int &sample_face_num, const std::string &save_mesh_file_path, const bool &overwrite){
  if (!std::filesystem::exists(mesh_file_path)){
    std::cout << "[ERROR][sample::sampleLocalSurfaceNearVertex]" << std::endl;
    std::cout << "\t mesh file not exist!" << std::endl;
    std::cout << "\t mesh_file_path : " << mesh_file_path << std::endl;
    return false;
  }

  if (std::filesystem::exists(save_mesh_file_path)){
    if (!overwrite){
      return true;
    }

    std::filesystem::remove(save_mesh_file_path);
  }

  open3d::geometry::TriangleMesh mesh;
  if (!open3d::io::ReadTriangleMesh(mesh_file_path, mesh)){
    std::cout << "[ERROR][sample::sampleLocalSurfaceNearVertex]" << std::endl;
    std::cout << "\t ReadTriangleMesh failed!" << std::endl;
    std::cout << "\t mesh_file_path : " << mesh_file_path << std::endl;
    return false;
  }

  int valid_vertex_idx = vertex_idx % mesh.vertices_.size();

  const auto &vertices = mesh.vertices_;
  const auto &triangles = mesh.triangles_;

  std::vector<int> unused_triangle_idxs;
  std::vector<int> selected_triangle_idxs;
  std::vector<int> selected_vertex_idxs;

  std::vector<int> current_search_vertex_idxs;
  std::vector<int> next_search_vertex_idxs;

  unused_triangle_idxs.reserve(triangles.size());
  for (int i = 0; i < triangles.size(); ++i) {
    unused_triangle_idxs.emplace_back(i);
  }

  current_search_vertex_idxs.emplace_back(valid_vertex_idx);

  bool is_sample_enough_faces = false;
  while (unused_triangle_idxs.size() > 0) {
    for (int i = 0; i < current_search_vertex_idxs.size(); ++i){
      const int &current_search_vertex_idx = current_search_vertex_idxs[i];

      for (int j = unused_triangle_idxs.size() - 1; j >= 0; --j){
        const int &unused_triangle_idx = unused_triangle_idxs[j];
        const Eigen::Vector3i &unused_triangle = triangles[unused_triangle_idx];

        if (unused_triangle[0] == current_search_vertex_idx ||
            unused_triangle[1] == current_search_vertex_idx ||
            unused_triangle[2] == current_search_vertex_idx) {
          selected_triangle_idxs.emplace_back(unused_triangle_idx);

          for (int k = 0; k < 3; ++k){
            const int selected_triangle_vertex_idx = unused_triangle[k];

            if (std::find(selected_vertex_idxs.begin(), selected_vertex_idxs.end(), selected_triangle_vertex_idx) == selected_vertex_idxs.end()){
              selected_vertex_idxs.emplace_back(selected_triangle_vertex_idx);
              next_search_vertex_idxs.emplace_back(selected_triangle_vertex_idx);
            }
          }

          unused_triangle_idxs.erase(unused_triangle_idxs.begin() + j);

          if (selected_triangle_idxs.size() >= sample_face_num){
            is_sample_enough_faces = true;
            break;
          }
        }
      }

      if (is_sample_enough_faces){
        break;
      }
    }

    if (is_sample_enough_faces){
      break;
    }
    if (next_search_vertex_idxs.size()==0) {
      break;
    }

    current_search_vertex_idxs = next_search_vertex_idxs;
    next_search_vertex_idxs.clear();
  }

  std::unordered_map<int, int> vertex_idx_map;
  
  std::vector<Eigen::Vector3d> selected_vertices;
  std::vector<Eigen::Vector3i> selected_triangles;

  for (int i = 0; i < selected_vertex_idxs.size(); ++i){
    const int selected_vertex_idx = selected_vertex_idxs[i];

    selected_vertices.emplace_back(vertices[selected_vertex_idx]);

    vertex_idx_map[selected_vertex_idx] = i;
  }

  for (int i = 0; i < selected_triangle_idxs.size(); ++i){
    const Eigen::Vector3i selected_triangle = triangles[selected_triangle_idxs[i]];

    const Eigen::Vector3i map_triangle = Eigen::Vector3i(
        vertex_idx_map[selected_triangle[0]],
        vertex_idx_map[selected_triangle[1]],
        vertex_idx_map[selected_triangle[2]]);
    selected_triangles.emplace_back(map_triangle);
  }

  open3d::geometry::TriangleMesh local_surface_mesh;
  local_surface_mesh.vertices_ = selected_vertices;
  local_surface_mesh.triangles_ = selected_triangles;

  open3d::io::WriteTriangleMesh(save_mesh_file_path, local_surface_mesh);
  
  return true;
}
