from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.filter import bilateral_filter
from open3d_manage.Metric.chamfer import chamferDistance
from open3d_manage.Method.render import renderGeometries

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
    curvature_weight_filter = True
    print_progress = True

    noise_pcd_file_path = pcd_path_dict["plane_noise-li_mac"]
    gt_pcd_file_path = pcd_path_dict["gt_plane-li_mac"]

    print("start loadGeometry...")
    noise_pcd = loadGeometry(
        noise_pcd_file_path,
        "pcd",
        print_progress,
    )
    assert noise_pcd is not None

    gt_pcd = loadGeometry(gt_pcd_file_path, "pcd", print_progress)
    assert gt_pcd is not None

    print("start bilateral_filter...")
    filter_pcd = bilateral_filter(
        noise_pcd, sigma_d, sigma_n, knn_num, curvature_weight_filter, print_progress
    )

    print("start chamferDistance...")
    chamfer_distance = chamferDistance(noise_pcd, gt_pcd)

    window_name = (
        "sigma_d="
        + str(sigma_d)
        + " sigma_n="
        + str(sigma_n)
        + " knn_num="
        + str(knn_num)
        + " CD="
        + str(chamfer_distance)
    )

    noise_pcd.translate([10, 0, 0])

    filter_pcd.translate([20, 0, 0])

    renderGeometries([gt_pcd, noise_pcd, filter_pcd], window_name)
    return True
