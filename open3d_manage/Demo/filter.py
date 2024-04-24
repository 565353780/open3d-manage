from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.curvature import (
    estimate_curvature_eig,
    estimate_curvature_fit,
)
from open3d_manage.Method.filter import bilateral_filter
from open3d_manage.Metric.chamfer import chamferDistance
from open3d_manage.Method.render import (
    renderGeometries,
    visualize_curvature,
    toFilterWeights,
)

pcd_path_dict = {
    "plane_noise-liu_win": "D:\\Program Files\\dev_for_python\\data\\plane_noise.pcd",
    "gt_plane-liu_win": "D:\\Program Files\\dev_for_python\\data\\plane.pcd",
    "plane_noise-li_mac": "../output_noise_pcd/airplane_gaussMean-100.0_gaussSigma-100.0_pcd.ply",
    "gt_plane-li_mac": "../output_noise_pcd/airplane_pcd.ply",
    "plane_noise-server": "./output/input_pcd/airplane_gaussMean-100.0_gaussSigma-100.0_pcd.ply",
    "gt_plane-server": "./output/input_pcd/airplane_pcd.ply",
}


def demo():
    sigma_d = 200.0
    sigma_n = 2000.0
    curvature_knn_num = 10
    filter_knn_num = 40
    print_progress = True

    system = ["liu_win", "li_mac", "server"][2]
    noise_pcd_file_path = pcd_path_dict["plane_noise-" + system]
    gt_pcd_file_path = pcd_path_dict["gt_plane-" + system]

    print("start loadGeometry...")
    noise_pcd = loadGeometry(
        noise_pcd_file_path,
        "pcd",
        print_progress,
    )
    assert noise_pcd is not None
    noise_pcd = noise_pcd.uniform_down_sample(10)

    gt_pcd = loadGeometry(gt_pcd_file_path, "pcd", print_progress)
    assert gt_pcd is not None
    gt_pcd = gt_pcd.uniform_down_sample(10)

    print("start estimate curvature by fit...")
    curvatures = estimate_curvature_fit(noise_pcd, curvature_knn_num, print_progress)
    weights = toFilterWeights(abs(curvatures))

    print("start bilateral_filter...")
    filter_pcd = bilateral_filter(
        noise_pcd, sigma_d, sigma_n, filter_knn_num, weights, print_progress
    )

    print("start chamferDistance...")
    noise_chamfer_distance = chamferDistance(noise_pcd, gt_pcd)
    filter_chamfer_distance = chamferDistance(filter_pcd, gt_pcd)

    print("noise_chamfer_distance:", noise_chamfer_distance)
    print("filter_chamfer_distance:", filter_chamfer_distance)

    window_name = (
        " sigma_n="
        + str(sigma_n)
        + " curvature_knn_num="
        + str(curvature_knn_num)
        + " filter_knn_num="
        + str(filter_knn_num)
        + " NoiseCD="
        + str(noise_chamfer_distance)
        + " FilterCD="
        + str(filter_chamfer_distance)
    )

    visualize_curvature(noise_pcd, curvatures)

    sizes = noise_pcd.get_axis_aligned_bounding_box().get_extent() * 1.1

    gt_pcd.translate([0, 0, -sizes[2]])

    filter_pcd.translate([0, 0, sizes[2]])

    renderGeometries([gt_pcd, noise_pcd, filter_pcd], window_name)
    return True
