#include "Curvature/curvature_estimator.h"
#include "Curvature/io.h"
#include "Curvature/render.h"
#include "MeshSplit/mesh_spliter.h"
#include "filter.h"
#include "noise_dataset_loader.h"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <limits>
#include <memory>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <string>

int denoisePointCloud() {
  // super params
  const float sigma_d = 200;
  const float sigma_n = 2000;
  const int curvature_knn_num = 10;
  const int filter_knn_num = 40;
  const bool need_smooth = true;

  // call data from dataset
  const std::string dataset_root_folder_path = "/home/chli/Dataset/";

  NoiseDatasetLoader noise_dataset_loader(dataset_root_folder_path);

  const std::vector<std::string> shape_id_vec =
      noise_dataset_loader.getShapeIdVec();

  const std::string shape_id = shape_id_vec[0];
  const std::string noise_type = "Gauss";
  std::unordered_map<std::string, std::string> params;

  params["sample_point_num"] = "10000"; // 10000, 100000
  if (noise_type == "Random") {
    params["strength"] = "0.005"; // 0.005, 0.01, 0.02
  } else if (noise_type == "Gauss") {
    params["mean"] = "0.0";    // 0.0
    params["sigma"] = "0.005"; // 0.005, 0.01, 0.02
  } else if (noise_type == "Impulse") {
    params["strength"] = "0.02";   // 0.02, 0.04, 0.08
    params["probability"] = "0.1"; // 0.1
  } else {
    std::cout << "noise type not defined!" << std::endl;
    return -1;
  }

  // input point cloud [x1, y1, z1, x2, y2, z2, ...]
  const std::vector<float> points =
      noise_dataset_loader.getNoisePoints(shape_id, noise_type, params);

  // filter method call
  const std::vector<float> denoised_points = toDenoisedPts(
      points, sigma_d, sigma_n, curvature_knn_num, filter_knn_num, need_smooth);

  // result output demo
  const int point_num = int(points.size() / 3);

  std::cout << "input points size: " << point_num << std::endl;
  std::cout << "denoised points size: " << denoised_points.size() / 3
            << std::endl;

  float min_move_dist = std::numeric_limits<float>().max();
  float mean_move_dist = 0.0;
  float max_move_dist = 0.0;
  std::vector<float> move_dists;
  move_dists.reserve(point_num);
  for (int i = 0; i < point_num; ++i) {
    const float x_diff = denoised_points[3 * i] - points[3 * i];
    const float y_diff = denoised_points[3 * i + 1] - points[3 * i + 1];
    const float z_diff = denoised_points[3 * i + 2] - points[3 * i + 2];

    const float move_dist = x_diff * x_diff + y_diff * y_diff + z_diff * z_diff;

    move_dists.emplace_back(move_dist);

    if (move_dist < min_move_dist) {
      min_move_dist = move_dist;
    }

    if (move_dist > max_move_dist) {
      max_move_dist = move_dist;
    }

    mean_move_dist += move_dist;
  }
  mean_move_dist /= move_dists.size();

  std::cout << "move dist : min " << min_move_dist << ", mean "
            << mean_move_dist << ", max " << max_move_dist << std::endl;

  return 1;
}

int estimateCurvature() {
  const std::string mesh_file_path = "../data/cow.ply";
  const std::string pcd_file_path = "../data/cow_points.ply";

  CurvatureEstimator curvature_estimator;

  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
      loadMeshFile(mesh_file_path);

  if (mesh_ptr->IsEmpty()) {
    std::cout << " loadMeshFile failed!" << std::endl;
    return -1;
  }

  const Eigen::VectorXd mesh_curvatures =
      curvature_estimator.toMeshTotalCurvature(mesh_ptr);
  if (mesh_curvatures.size() == 0) {
    std::cout << " toMeshTotalCurvature failed!" << std::endl;
    return -1;
  }

  renderMeshCurvature(mesh_ptr, mesh_curvatures);

  std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr =
      loadPcdFile(pcd_file_path);

  if (pcd_ptr->IsEmpty()) {
    std::cout << " loadPcdFile failed!" << std::endl;
    return -1;
  }

  const Eigen::VectorXd pcd_curvatures =
      curvature_estimator.toPcdTotalCurvature(pcd_ptr);

  if (pcd_curvatures.size() == 0) {
    std::cout << " toPcdTotalCurvature failed!" << std::endl;
    return -1;
  }

  renderPointCloudCurvature(pcd_ptr, pcd_curvatures);

  return 1;
}

int splitMesh() {
  std::string mesh_file_path = "../data/cow.ply";
  mesh_file_path = "/home/chli/Dataset/ManifoldMesh/ShapeNet/02691156/"
                   "1026dd1b26120799107f68a9cb8e3c.obj";
  mesh_file_path = "/home/chli/Dataset/ManifoldMesh/ShapeNet-Crop/02691156/"
                   "10155655850468db78d106ce0a280f87-crop.obj";
  mesh_file_path = "/home/chli/Dataset/ManifoldMesh/ShapeNet-Crop/02691156/"
                   "105f7f51e4140ee4b6b87e72ead132ed-crop.obj";
  const int max_merge_curvature = 2000.0;
  const std::string save_folder_path = "./output/sub_meshes/";
  const bool overwrite = true;

  CurvatureEstimator curvature_estimator;
  MeshSpliter mesh_spliter;

  std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr =
      loadMeshFile(mesh_file_path);

  if (mesh_ptr->IsEmpty()) {
    std::cout << " loadMeshFile failed!" << std::endl;
    return -1;
  }

  const Eigen::VectorXd mesh_curvatures =
      curvature_estimator.toMeshTotalCurvature(mesh_ptr);
  if (mesh_curvatures.size() == 0) {
    std::cout << " toMeshTotalCurvature failed!" << std::endl;
    return -1;
  }

  // renderMeshCurvature(mesh_ptr, mesh_curvatures);

  const std::unordered_map<int, std::set<int>> sub_mesh_face_idx_set_map =
      mesh_spliter.splitMeshByCurvature(mesh_ptr, mesh_curvatures,
                                        max_merge_curvature);

  mesh_spliter.saveSubMeshes(mesh_ptr, sub_mesh_face_idx_set_map,
                             save_folder_path, overwrite);
  return 1;
}

int main() {
  // denoisePointCloud();

  // estimateCurvature();

  splitMesh();

  return 1;
}
