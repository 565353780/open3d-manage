import numpy as np
import open3d as o3d
from tqdm import tqdm

from open3d_manage.Method.knn import toKNNIdxs


# 利用协方差矩阵特征值求曲率
def estimate_curvature_eig(
    pcd: o3d.geometry.PointCloud, k_num: int = 30, print_progress: bool = False
) -> np.ndarray:
    """by Junyi Liu"""
    pcd.estimate_covariances(o3d.geometry.KDTreeSearchParamKNN(k_num))
    covariances = np.asarray(pcd.covariances)

    point_num = np.asarray(pcd.points).shape[0]

    curvatures = np.zeros(point_num)

    for_data = range(point_num)
    if print_progress:
        print("[INFO][curvatures::estimate_curvature_eig]")
        print("\t start estimate curvature eig for each point...")
        for_data = tqdm(for_data)
    for i in for_data:
        eigval = np.linalg.eigvals(covariances[i])
        curvatures[i] = eigval.min() / eigval.sum()

    return curvatures


# 利用二次曲面拟合求曲率
def estimate_curvature_fit(
    pcd: o3d.geometry.PointCloud, k_num: int = 30, print_progress: bool = False
) -> np.ndarray:
    """by Junyi Liu"""
    points = np.asarray(pcd.points)

    idxs = toKNNIdxs(points, k_num)

    curvatures = np.zeros(points.shape[0])

    for_data = range(points.shape[0])
    if print_progress:
        print("[INFO][curvatures::estimate_curvature_fit]")
        print("\t start estimate curvature fit for each point...")
        for_data = tqdm(for_data)
    for i in for_data:
        idx = idxs[i]

        A = np.zeros((k_num, 6))
        b = np.zeros((k_num, 1))

        for j in range(k_num):
            p = points[idx[j]]
            A[j, :] = np.array([p[0] ** 2, p[0] * p[1], p[1] ** 2, p[0], p[1], 1])
            b[j] = p[2]

        x = np.linalg.lstsq(A, b, rcond=None)[0].reshape(-1)

        # 求曲率
        point = points[i]

        rx = np.array(
            [1.0, 0.0, 2.0 * x[0] * point[0] + x[1] * point[1] + x[3]]
        ).astype(float)
        ry = np.array(
            [0.0, 1.0, 2.0 * x[2] * point[1] + x[1] * point[0] + x[4]]
        ).astype(float)
        rxx = np.array([0.0, 0.0, 2.0 * x[0]]).astype(float)
        rxy = np.array([0.0, 0.0, x[1]]).astype(float)
        ryy = np.array([0.0, 0.0, 2.0 * x[2]]).astype(float)

        k1 = np.linalg.norm(rx)**2 * np.linalg.norm(ry)**2 - np.dot(rx, ry) ** 2 #修改了第一基本形式中的E和G为norm平方
        n = np.cross(rx, ry) / np.linalg.norm(np.cross(rx, ry))
        k2 = np.dot(rxx, n) * np.dot(ryy, n) - np.dot(rxy, n) ** 2

        curvatures[i] = k2 / (k1 + 1e-6)

    return curvatures
