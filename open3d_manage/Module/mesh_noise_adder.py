import os
import open3d as o3d
from copy import deepcopy

from open3d_manage.Method.noise import toRandomNoisePCD, toGaussNoisePCD, toImpulseNoisePCD
from open3d_manage.Method.path import removeFile, createFileFolder

class MeshNoiseAdder:
    def __init__(self, mesh: o3d.geometry.TriangleMesh) -> None:
        self.mesh = deepcopy(mesh)

        self.sampled_pcd_dict = {}
        self.pcd = None
        return

    def samplePcd(self, sample_point_num: int) -> bool:
        if str(sample_point_num) in self.sampled_pcd_dict.keys():
            return True

        sample_pcd = self.mesh.sample_points_uniformly(sample_point_num)
        # sample_pcd = self.mesh.sample_points_poisson_disk(sample_point_num)

        self.sampled_pcd_dict[str(sample_point_num)] = sample_pcd
        return True

    def addRandomNoise(self, sample_point_num: int, strength: float) -> o3d.geometry.PointCloud:
        self.pcd = None

        if not self.samplePcd(sample_point_num):
            print('[ERROR][MeshNoiseAdder::addRandomNoise]')
            print('\t samplePcd failed!')
            return False

        sample_pcd = self.sampled_pcd_dict[str(sample_point_num)]

        self.pcd = toRandomNoisePCD(sample_pcd, strength)
        return True

    def addGaussNoise(self, sample_point_num: int, mean: float, sigma: float) -> o3d.geometry.PointCloud:
        self.pcd = None

        if not self.samplePcd(sample_point_num):
            print('[ERROR][MeshNoiseAdder::addGaussNoise]')
            print('\t samplePcd failed!')
            return False

        sample_pcd = self.sampled_pcd_dict[str(sample_point_num)]

        self.pcd = toGaussNoisePCD(sample_pcd, mean, sigma)
        return True

    def addImpulseNoise(self, sample_point_num: int, strength: float, probability: float) -> o3d.geometry.PointCloud:
        self.pcd = None

        if not self.samplePcd(sample_point_num):
            print('[ERROR][MeshNoiseAdder::addImpulseNoise]')
            print('\t samplePcd failed!')
            return False

        sample_pcd = self.sampled_pcd_dict[str(sample_point_num)]

        self.pcd = toImpulseNoisePCD(sample_pcd, strength, probability)
        return True

    def save(self, save_file_path: str, overwrite: bool = False) -> bool:
        if self.pcd is None:
            print('[ERROR][MeshNoiseAdder::save]')
            print('\t pcd not valid!')
            return False

        if os.path.exists(save_file_path):
            if not overwrite:
                return True

            removeFile(save_file_path)

        createFileFolder(save_file_path)

        o3d.io.write_point_cloud(save_file_path, self.pcd, write_ascii=True)

        return True
