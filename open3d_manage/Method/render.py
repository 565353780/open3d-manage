import numpy as np
import open3d as o3d
from copy import deepcopy


def renderGeometries(geometries, window_name: str = "Open3D Renderer") -> bool:
    if not isinstance(geometries, list):
        geometries = [geometries]

    o3d.visualization.draw_geometries(geometries, window_name=window_name)
    return True


def visualize_curvature(pcd: o3d.geometry.PointCloud, curvatures: np.ndarray) -> bool:
    """by Junyi Liu"""
    render_pcd = deepcopy(pcd)

    point_num = np.asarray(render_pcd.points).shape[0]

    abs_curvatures = np.abs(curvatures)

    min = np.min(abs_curvatures)
    max = np.max(abs_curvatures)
    print("curvatures range:", min, max)
    min = 0
    max = 10000

    curvature_colors = np.zeros((point_num, 3))

    for point_idx in range(point_num):
        # colormap
        abs_curvature_value = abs_curvatures[point_idx]
        ratio = (abs_curvature_value - min) / (max - min)

        if 0 <= ratio < 0.25:
            curvature_colors[point_idx] = [0, ratio * 4, 1]
        elif ratio < 0.5:
            curvature_colors[point_idx] = [0, 1, 1 - (ratio - 0.25) * 4]
        elif ratio < 0.75:
            curvature_colors[point_idx] = [(ratio - 0.5) * 4, 1, 0]
        elif ratio < 1:
            curvature_colors[point_idx] = [1, 1 - (ratio - 0.75) * 4, 0]
        else:
            curvature_colors[point_idx] = [1, 0, 0]

    render_pcd.colors = o3d.utility.Vector3dVector(curvature_colors)

    renderGeometries(render_pcd, "curvature")
    return True
