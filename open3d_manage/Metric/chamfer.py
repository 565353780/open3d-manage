import numpy as np


def chamferDistance(pcd1, pcd2):
    """by Junyi Liu"""
    dist1 = np.asarray(pcd1.compute_point_cloud_distance(pcd2))
    dist2 = np.asarray(pcd2.compute_point_cloud_distance(pcd1))
    return np.mean(dist1) + np.mean(dist2)
