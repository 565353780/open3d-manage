#include "MeshSplit/mesh_spliter.h"
#include "MeshSplit/idx_curvature.h"
#include "MeshSplit/sub_mesh_manager.h"
#include "MeshSplit/trans.h"
#include <algorithm>

const bool MeshSpliter::splitMeshByCurvature(
    std::shared_ptr<open3d::geometry::TriangleMesh> &mesh_ptr,
    const Eigen::VectorXd &mesh_curvatures, const float &max_merge_curvature) {
  const int vertex_num = mesh_ptr->vertices_.size();

  if (vertex_num != mesh_curvatures.size()) {
    std::cout << "[ERROR][MeshSpliter::splitMeshByCurvature]" << std::endl;
    std::cout << "\t mesh vertex num != mesh curvatures num!" << std::endl;
    return false;
  }

  std::vector<double> curvatures_vec(mesh_curvatures.data(),
                                     mesh_curvatures.data() + vertex_num);

  std::vector<IdxCurvature> unused_curvatures;
  unused_curvatures.reserve(vertex_num);

  for (int i = 0; i < vertex_num; ++i) {
    unused_curvatures.emplace_back(IdxCurvature(i, mesh_curvatures[i]));
  }

  std::sort(
      unused_curvatures.begin(), unused_curvatures.end(),
      [](IdxCurvature a, IdxCurvature b) { return a.curvature > b.curvature; });

  TriMesh mesh = toOpenMesh(mesh_ptr);

  SubMeshManager sub_mesh_manager(mesh);

  // 按照曲率从小到大逐点检查
  while (!unused_curvatures.empty()) {
    const IdxCurvature current_unused_idx_curvature = unused_curvatures.back();
    unused_curvatures.pop_back();

    const int current_vertex_idx = current_unused_idx_curvature.idx;

    sub_mesh_manager.addVertexIntoSubSet(current_vertex_idx, curvatures_vec,
                                         max_merge_curvature);

    std::cout << unused_curvatures.size() << std::endl;
  }

  std::cout << sub_mesh_manager.sub_mesh_face_idx_set_map.size() << std::endl;

  return true;
}
