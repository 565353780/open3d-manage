import numpy as np
import open3d as o3d
from typing import Union

def toNormals(points: Union[np.ndarray, list], knn_num: int = 0, need_smooth: bool = False) -> np.ndarray:
    if isinstance(points, list):
        points = np.array(points)

    is_flatten = len(points.shape) == 1

    if is_flatten:
        points = points.reshape(-1, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if knn_num > 0:
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(knn_num))
    else:
        pcd.estimate_normals()

    pcd.normalize_normals()

    if need_smooth > 0:
        pcd.orient_normals_consistent_tangent_plane(knn_num)

    normals = np.asarray(pcd.normals)

    if is_flatten:
        normals = normals.reshape(-1)

    return normals

def toNormalsList(points: list, knn_num: int = 0, need_smooth: bool = False) -> list:
    return toNormals(points, knn_num, need_smooth).tolist()
