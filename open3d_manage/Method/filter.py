import numpy as np
import open3d as o3d
from tqdm import tqdm
from typing import Union
from copy import deepcopy


def bilateral_filter(
    pcd: o3d.geometry.PointCloud,
    sigma_d: float,
    sigma_n: float,
    knn_num: int,
    curvatures: Union[np.ndarray, None] = None,
    print_progress: bool = False,
):
    """by Junyi Liu"""
    filter_pcd = deepcopy(pcd)

    # 构建kdtree
    pcd_tree = o3d.geometry.KDTreeFlann(filter_pcd)

    # 估计法线
    filter_pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn_num))
    filter_pcd.normalize_normals()
    filter_pcd.orient_normals_consistent_tangent_plane(knn_num)

    points = np.asarray(filter_pcd.points)
    normals = np.asarray(filter_pcd.normals)
    if curvatures is None:
        curvatures = np.ones_like(points)

    curvatures_weights = 1.0 / (curvatures + 1e-6)
    curvatures_weights /= np.max(curvatures_weights)

    for_data = range(points.shape[0])
    if print_progress:
        print("[INFO][filter::bilateral_filter]")
        print("\t start bilateral filter for each point...")
        for_data = tqdm(for_data)
    # 双边滤波
    for point_idx in for_data:
        # k近邻点
        [_, idx, _] = pcd_tree.search_knn_vector_3d(points[point_idx], knn_num)

        sum_weight = 0
        sum_lambda = 0
        for neighbor_idx in idx[1:]:
            # 计算权重
            distance = np.linalg.norm(points[neighbor_idx] - points[point_idx])

            normal_dot = np.dot(
                normals[point_idx], points[neighbor_idx] - points[point_idx]
            )

            weight = np.exp(-(distance**2) / (2 * sigma_d**2)) * np.exp(
                -(normal_dot**2) / (2 * sigma_n**2)
            )

            sum_weight += weight
            sum_lambda += weight * normal_dot

        # FIXME: here sum weight may be 0 because
        # -(distance**2) / (2 * sigma_d**2)
        # is too small! e.g. -9000
        if sum_lambda == 0 or sum_weight == 0:
            continue

        # 更新点云
        filter_pcd.points[point_idx] += (
            sum_lambda / sum_weight * normals[point_idx] * curvatures_weights[point_idx]
        )

    return filter_pcd
