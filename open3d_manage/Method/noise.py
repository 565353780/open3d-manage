import numpy as np
import open3d as o3d
from copy import deepcopy


def getGaussNoise(mean: float, sigma: float, noise_shape: list) -> np.ndarray:
    return np.random.normal(mean, sigma, noise_shape)

def getImpulseNoise(probability: float, noise_shape: list) -> np.ndarray:
    noise = np.zeros(noise_shape, dtype=np.float32)

    noise[np.random.rand(*noise_shape) < probability] = np.random.rand()
    return noise


def toGaussNoisePCD(
    pcd: o3d.geometry.PointCloud, mean: float, sigma: float
) -> o3d.geometry.PointCloud:
    gauss_noise_pcd = deepcopy(pcd)

    points = np.asarray(gauss_noise_pcd.points)

    gauss_noise = getGaussNoise(mean, sigma, points.shape)
    points += gauss_noise

    gauss_noise_pcd.points = o3d.utility.Vector3dVector(points)
    return gauss_noise_pcd

 
def toImpulseNoisePCD(pcd: o3d.geometry.PointCloud, probability: float):
    impulse_noise_pcd = deepcopy(pcd)
 
    points = np.asarray(impulse_noise_pcd.points)

    impulse_noise = getImpulseNoise(probability, points.shape)
    points += impulse_noise

    impulse_noise_pcd.points = o3d.utility.Vector3dVector(points)
    return impulse_noise_pcd
