import open3d as o3d
import numpy as np
from open3d_manage.Method.curvature import estimate_curvature_fit
from open3d_manage.Method.render import visualize_curvature
from open3d_manage.Method.filter import bilateral_filter
from open3d_manage.Method.noise import toGaussNoisePCD


if __name__ == "__main__":
    pcd = o3d.io.read_point_cloud("D:\\Program Files\\dev_for_python\\data\\plane_noise.pcd")

    # c = abs(estimate_curvature_fit(pcd, 15))

    # visualize_curvature(pcd, c)

    o3d.visualization.draw_geometries([pcd])

    

