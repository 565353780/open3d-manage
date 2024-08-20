import os
import open3d as o3d
from copy import deepcopy

from open3d_manage.Method.noise import toRandomNoisePCD, toGaussNoisePCD, toImpulseNoisePCD
from open3d_manage.Method.path import removeFile, createFileFolder

class NoiseAdder:
    def __init__(self, pcd: o3d.geometry.PointCloud) -> None:
        self.source_pcd = deepcopy(pcd)
        self.pcd = deepcopy(pcd)
        return

    def reset(self) -> bool:
        self.pcd = deepcopy(self.source_pcd)
        return True

    def addRandomNoise(self, strength: float) -> o3d.geometry.PointCloud:
        self.pcd = toRandomNoisePCD(self.pcd, strength)
        return True

    def addGaussNoise(self, mean: float, sigma: float) -> o3d.geometry.PointCloud:
        self.pcd = toGaussNoisePCD(self.pcd, mean, sigma)
        return True

    def addImpulseNoise(self, strength: float, probability: float) -> o3d.geometry.PointCloud:
        self.pcd = toImpulseNoisePCD(self.pcd, strength, probability)
        return True

    def save(self, save_file_path: str, overwrite: bool = False, need_reset: bool = True) -> bool:
        if os.path.exists(save_file_path):
            if not overwrite:
                return True

            removeFile(save_file_path)

        createFileFolder(save_file_path)

        o3d.io.write_point_cloud(save_file_path, self.pcd, write_ascii=True)

        if need_reset:
            self.reset()
        return True
