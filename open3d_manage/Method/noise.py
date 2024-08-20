import numpy as np
import open3d as o3d
from copy import deepcopy

def getRandomNoise(strength: float, noise_shape: list) -> np.ndarray:
    return strength * np.random.uniform(-1, 1, noise_shape)

def getGaussNoise(mean: float, sigma: float, noise_shape: list) -> np.ndarray:
    return np.random.normal(mean, sigma, noise_shape)

def getImpulseNoise(strength: float, probability: float, noise_shape: list) -> np.ndarray:
    noise = np.zeros(noise_shape, dtype=np.float32)
    noise[np.random.rand(*noise_shape) < probability] = strength * np.random.rand()
    return noise

def toRandomNoisePCD(
    pcd: o3d.geometry.PointCloud, strength: float) -> o3d.geometry.PointCloud:
    random_noise_pcd = deepcopy(pcd)

    points = np.asarray(random_noise_pcd.points)

    random_noise = getRandomNoise(strength, points.shape)
    points += random_noise

    random_noise_pcd.points = o3d.utility.Vector3dVector(points)
    return random_noise_pcd


def toGaussNoisePCD(
    pcd: o3d.geometry.PointCloud, mean: float, sigma: float
) -> o3d.geometry.PointCloud:
    gauss_noise_pcd = deepcopy(pcd)

    points = np.asarray(gauss_noise_pcd.points)

    gauss_noise = getGaussNoise(mean, sigma, points.shape)
    points += gauss_noise

    gauss_noise_pcd.points = o3d.utility.Vector3dVector(points)
    return gauss_noise_pcd

 
def toImpulseNoisePCD(pcd: o3d.geometry.PointCloud, strength: float, probability: float):
    impulse_noise_pcd = deepcopy(pcd)
 
    points = np.asarray(impulse_noise_pcd.points)

    impulse_noise = getImpulseNoise(strength, probability, points.shape)
    points += impulse_noise

    impulse_noise_pcd.points = o3d.utility.Vector3dVector(points)
    return impulse_noise_pcd
