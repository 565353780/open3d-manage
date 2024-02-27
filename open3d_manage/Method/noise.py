import numpy as np
import open3d as o3d
from copy import deepcopy


def getGaussNoise(mean: float, sigma: float, noise_shape: list) -> np.ndarray:
    return np.random.normal(mean, sigma, noise_shape)


def toGaussNoisePCD(
    pcd: o3d.geometry.PointCloud, mean: float, sigma: float
) -> o3d.geometry.PointCloud:
    gauss_noise_pcd = deepcopy(pcd)

    points = np.asarray(gauss_noise_pcd.points)

    gauss_noise = getGaussNoise(mean, sigma, points.shape)
    points += gauss_noise

    gauss_noise_pcd.points = o3d.utility.Vector3dVector(points)
    return gauss_noise_pcd
