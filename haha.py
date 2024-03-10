import open3d as o3d
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt


if __name__ == "__main__":
    # pcd = o3d.io.read_point_cloud("D:\\Program Files\\dev_for_python\\data\\plane_noise.pcd")
    # pcd_source = o3d.io.read_point_cloud("D:\\Program Files\\dev_for_python\\data\\plane.pcd")

    # sigma_d = 0.2
    # sigma_n = 10
    # knn_num = 120

    # bilateral_filter(pcd, sigma_d, sigma_n, knn_num)

    # window_name = "sigma_d=" + str(sigma_d) + " sigma_n=" + str(sigma_n) + " knn_num=" + str(knn_num) + " Chamfer Distance=" + str(chamfer_distance(pcd, pcd_source))

    # o3d.visualization.draw_geometries([pcd], window_name)
    # -------------------------------------------------------------------
    pcd = o3d.io.read_point_cloud(
        "D:\\Program Files\\dev_for_python\\data\\result\\plane_noise.pcd"
    )

    a = estimate_curvature(pcd)

    print(a.max())
    print(a.min())
    print(abs(a).min())

    visualize_curvature(pcd, a)
