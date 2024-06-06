import numpy as np
import open3d as o3d
from typing import Union

def toKNNIdxs(points: Union[np.ndarray, list], knn_num: int) -> np.ndarray:
    if isinstance(points, list):
        points = np.array(points)

    is_flatten = len(points.shape) == 1

    if is_flatten:
        points = points.reshape(-1, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    knn_idxs = []
    for i in range(points.shape[0]):
        [_, knn_idx, _] = pcd_tree.search_knn_vector_3d(points[i], knn_num)
        knn_idxs.append(knn_idx)

    knn_idxs = np.array(knn_idxs, dtype=np.int64)

    if is_flatten:
        knn_idxs = knn_idxs.reshape(-1)

    return knn_idxs

def toKNNIdxsList(points: list, knn_num: int) -> list:
    return toKNNIdxs(points, knn_num).tolist()
