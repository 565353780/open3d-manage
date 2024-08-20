import open3d as o3d
from copy import deepcopy

from open3d_manage.Method.noise import toRandomNoisePCD, toGaussNoisePCD, toImpulseNoisePCD

class NoiseAdder:
    def __init__(self, pcd: o3d.geometry.PointCloud) -> None:
        self.pcd = deepcopy(pcd)
        return

    def addRandomNoise(self, strength: float) -> o3d.geometry.PointCloud:
        return toRandomNoisePCD(self.pcd, strength)

    def addGaussNoise(self, mean: float, sigma: float) -> o3d.geometry.PointCloud:
        return toGaussNoisePCD(self.pcd, mean, sigma)

    def addImpulseNoise(self, strength: float, probability: float) -> o3d.geometry.PointCloud:
        return toImpulseNoisePCD(self.pcd, strength, probability)
