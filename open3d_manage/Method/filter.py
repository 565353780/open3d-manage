import numpy as np
import open3d as o3d
from tqdm import tqdm
from typing import Union
from copy import deepcopy

from open3d_manage.Method.knn import toKNNIdxs
from open3d_manage.Method.normal import toNormals


def bilateral_filter(
    pcd: o3d.geometry.PointCloud,
    sigma_d: float,
    sigma_n: float,
    knn_num: int,
    curvatures_weight: Union[np.ndarray, None] = None,
    print_progress: bool = False,
):
    """by Junyi Liu"""
    points = np.asarray(pcd.points)
    normals = toNormals(points, knn_num, True)

    # k近邻点
    idxs = toKNNIdxs(points, knn_num)

    filter_pcd = deepcopy(pcd)

    if curvatures_weight is None:
        curvatures_weight = np.ones_like(points)

    for_data = range(points.shape[0])
    if print_progress:
        print("[INFO][filter::bilateral_filter]")
        print("\t start bilateral filter for each point...")
        for_data = tqdm(for_data)

    # 双边滤波
    for point_idx in for_data:
        idx = idxs[point_idx]

        sum_weight = 0
        sum_lambda = 0
        for neighbor_idx in idx[1:]:
            # 计算权重
            distance = np.linalg.norm(points[neighbor_idx] - points[point_idx])

            normal_dot = np.dot(
                normals[point_idx], points[neighbor_idx] - points[point_idx]
            )

            weight = np.exp(
                -(distance**2) / (2 * sigma_d**2) - (normal_dot**2) / (2 * sigma_n**2)
            )

            sum_weight += weight
            sum_lambda += weight * normal_dot

        # FIXME: here sum weight may be 0 because
        # -(distance**2) / (2 * sigma_d**2)
        # is too small! e.g. -9000
        if sum_lambda == 0 or sum_weight == 0:
            continue

        move_dist = sum_lambda / sum_weight * curvatures_weight[point_idx]

        # 更新点云
        filter_pcd.points[point_idx] += move_dist * normals[point_idx]

    return filter_pcd

def toFilterWeights(curvatures):
    # for i in range(30):
    #     std = np.std(curvatures)
    #     mean = np.mean(curvatures)
    #     print(f"std:{std}, mean:{mean}")

    #     dlimit = mean - 3 * std
    #     ulimit = mean + 3 * std

    #     curvatures = np.where(curvatures < dlimit, 0, curvatures)
    #     curvatures = np.where(curvatures > ulimit, ulimit, curvatures)
    # mean = np.mean(curvatures)
    # weights = np.ones_like(curvatures)
    # weights = np.where(curvatures >= mean, np.exp(-(curvatures-mean)**2/mean**2), weights)

    # -----------------箱线剔除离群点--------------------------
    Q1 = np.percentile(curvatures, 25)
    Q3 = np.percentile(curvatures, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    curvatures = np.where(curvatures < lower_bound, 0, curvatures)
    curvatures = np.where(curvatures > upper_bound, upper_bound, curvatures)
    mean = np.mean(curvatures)
    weights = np.ones_like(curvatures)
    weights = np.where(
        curvatures >= mean, np.exp(-((curvatures - mean) ** 2) / mean**2), weights
    )

    return weights
