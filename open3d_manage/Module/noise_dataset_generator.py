import os
import open3d as o3d
from tqdm import tqdm

from open3d_manage.Module.mesh_noise_adder import MeshNoiseAdder

class NoiseDatasetGenerator(object):
    def __init__(self, dataset_root_folder_path: str) -> None:
        self.dataset_root_folder_path = dataset_root_folder_path

        self.source_mesh_folder_path = self.dataset_root_folder_path + "ManifoldMesh/ShapeNet/02691156/"
        self.noise_pcd_folder_path = self.dataset_root_folder_path + "ManifoldMesh_NoisePcd/ShapeNet/02691156/"

        assert self.isValid()
        return

    def isValid(self) -> bool:
        if not os.path.exists(self.dataset_root_folder_path):
            print('[ERROR][NoiseDatasetGenerator::isValid]')
            print('\t dataset root folder not exist!')
            print('\t dataset_root_folder_path :', self.dataset_root_folder_path)
            return False

        if not os.path.exists(self.source_mesh_folder_path):
            print('[ERROR][NoiseDatasetGenerator::isValid]')
            print('\t source mesh folder not exist!')
            print('\t source_mesh_folder_path:', self.source_mesh_folder_path)
            return False

        return True

    def generateNoisePcdFromDict(self, mesh_noise_adder: MeshNoiseAdder, noise_dict: dict, save_noise_pcd_folder_path: str, overwrite: bool = False) -> bool:
        if 'sample_point_num' not in noise_dict:
            print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
            print('\t sample_point_num not defined!')
            return False

        sample_point_num = int(noise_dict['sample_point_num'])

        if 'type' not in noise_dict:
            print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
            print('\t type not defined!')
            return False

        noise_type = noise_dict['type']

        if noise_type == 'Random':
            if 'strength' not in noise_dict:
                print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
                print('\t RandomNoise strength not found!')
                return False

            strength = noise_dict['strength']

            save_noise_pcd_file_path = save_noise_pcd_folder_path + 'Random/sample-' + str(sample_point_num) + '_strength-' + str(strength) + '.ply'
            if not overwrite:
                if os.path.exists(save_noise_pcd_file_path):
                    return True

            mesh_noise_adder.addRandomNoise(sample_point_num, strength)

            mesh_noise_adder.save(save_noise_pcd_file_path, overwrite)

            return True

        if noise_type == 'Gauss':
            if 'mean' not in noise_dict:
                print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
                print('\t GaussNoise mean not found!')
                return False

            if 'sigma' not in noise_dict:
                print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
                print('\t GaussNoise sigma not found!')
                return False

            mean = noise_dict['mean']
            sigma = noise_dict['sigma']

            save_noise_pcd_file_path = save_noise_pcd_folder_path + 'Gauss/sample-' + str(sample_point_num) + '_mean-' + str(mean) + '_sigma-' + str(sigma) + '.ply'
            if not overwrite:
                if os.path.exists(save_noise_pcd_file_path):
                    return True

            mesh_noise_adder.addGaussNoise(sample_point_num, mean, sigma)

            mesh_noise_adder.save(save_noise_pcd_file_path, overwrite)

            return True

        if noise_type == 'Impulse':
            if 'strength' not in noise_dict:
                print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
                print('\t ImpulseNoise strength not found!')
                return False

            if 'probability' not in noise_dict:
                print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
                print('\t ImpulseNoise probability not found!')
                return False

            strength = noise_dict['strength']
            probability = noise_dict['probability']

            save_noise_pcd_file_path = save_noise_pcd_folder_path + 'Impulse/sample-' + str(sample_point_num) + '_strength-' + str(strength) + '_probability-' + str(probability) + '.ply'
            if not overwrite:
                if os.path.exists(save_noise_pcd_file_path):
                    return True

            mesh_noise_adder.addImpulseNoise(sample_point_num, strength, probability)

            mesh_noise_adder.save(save_noise_pcd_file_path, overwrite)

            return True

        print('[ERROR][NoiseDatasetGenerator::generateNoisePcdFromDict]')
        print('\t noist type not defined!')
        print('\t noise_type :', noise_type)
        return False

    def generateNoisePcds(self, mesh: o3d.geometry.TriangleMesh, noise_dict_list: list, save_noise_pcd_folder_path: str, overwrite: bool = False) -> bool:
        mesh_noise_adder = MeshNoiseAdder(mesh)

        for noise_dict in tqdm(noise_dict_list):
            if not self.generateNoisePcdFromDict(mesh_noise_adder, noise_dict, save_noise_pcd_folder_path, overwrite):
                print('[WARN][NoiseDatasetGenerator::generateNoisePcds]')
                print('\t generateNoisePcdFromDict failed!')

                continue

        return True

    def autoGenerateNoisePcds(self, noise_dict_list: list, overwrite: bool = False, skip_id_list: list = []) -> bool:
        mesh_filename_list = os.listdir(self.source_mesh_folder_path)
        mesh_filename_list.sort()

        mesh_num = len(mesh_filename_list)

        for i, mesh_filename in enumerate(mesh_filename_list):
            mesh_file_basename = mesh_filename.split('.')[0]

            if mesh_file_basename in skip_id_list:
                continue

            mesh_file_path = self.source_mesh_folder_path + mesh_filename

            mesh = o3d.io.read_triangle_mesh(mesh_file_path)

            save_noise_pcd_folder_path = self.noise_pcd_folder_path + mesh_file_basename + '/'

            print('[INFO][NoiseDatasetGenerator::autoGenerateNoisePcds]')
            print('\t start generate noise pcds ', i + 1, mesh_num, '...')
            if not self.generateNoisePcds(mesh, noise_dict_list, save_noise_pcd_folder_path, overwrite):
                continue

        return True
