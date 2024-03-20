from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.curvature import (
    estimate_curvature_eig,
    estimate_curvature_fit,
)
from open3d_manage.Method.filter import bilateral_filter
from open3d_manage.Metric.chamfer import chamferDistance
from open3d_manage.Method.render import renderGeometries, visualize_curvature, toFilterWeights

pcd_path_dict = {
    "plane_noise-liu_win": "D:\\Program Files\\dev_for_python\\data\\plane_noise.pcd",
    "gt_plane-liu_win": "D:\\Program Files\\dev_for_python\\data\\plane.pcd",
    "plane_noise-li_mac": "../output_noise_pcd/airplane_gaussMean-100.0_gaussSigma-100.0_pcd.ply",
    "gt_plane-li_mac": "../output_noise_pcd/airplane_pcd.ply",
}


def demo():
    sigma_d = 0.2
    sigma_n = 10
    knn_num = 30
    print_progress = True

    noise_pcd_file_path = pcd_path_dict["plane_noise-liu_win"]
    gt_pcd_file_path = pcd_path_dict["gt_plane-liu_win"]

    print("start loadGeometry...")
    noise_pcd = loadGeometry(
        noise_pcd_file_path,
        "pcd",
        print_progress,
    )
    assert noise_pcd is not None

    gt_pcd = loadGeometry(gt_pcd_file_path, "pcd", print_progress)
    assert gt_pcd is not None

    print("start estimate curvature by fit...")
    curvatures = estimate_curvature_fit(noise_pcd, knn_num, print_progress)
    weights = toFilterWeights(abs(curvatures))

    print("start bilateral_filter...")
    filter_pcd = bilateral_filter(
        noise_pcd, sigma_d, sigma_n, 120, weights, print_progress
    )

    print("start chamferDistance...")
    chamfer_distance = chamferDistance(filter_pcd, gt_pcd)

    window_name = (  
        " sigma_n="
        + str(sigma_n)
        + " knn_num="
        + str(knn_num)
        + " CD="
        + str(chamfer_distance)
    )

    visualize_curvature(noise_pcd, curvatures)

    noise_pcd.translate([0, 0.2, 0])

    filter_pcd.translate([0, -0.2, 0])

    renderGeometries([gt_pcd, noise_pcd, filter_pcd], window_name)
    # renderGeometries([noise_pcd], window_name)
    return True
