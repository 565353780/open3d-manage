import os
from tqdm import tqdm
from shutil import copyfile

from open3d_manage.Method.path import removeFile
from open3d_manage.Module.noise_dataset_generator import NoiseDatasetGenerator

def copy_dataset_to_new_folder(dataset_root_folder_path, new_dataset_root_folder_path, copy_file_num, skip_id_list):
    source_mesh_folder_path = dataset_root_folder_path + "ManifoldMesh/ShapeNet/02691156/"
    source_mesh_filename_list = os.listdir(source_mesh_folder_path)
    source_mesh_filename_list.sort()

    # only for checking failure case file name
    if False:
        print(source_mesh_filename_list[99])
        exit()

    if copy_file_num < 0:
        copy_file_num = len(source_mesh_filename_list)

    save_mesh_folder_path = new_dataset_root_folder_path + "ManifoldMesh/ShapeNet/02691156/"
    os.makedirs(save_mesh_folder_path, exist_ok=True)

    print('[INFO][noise_dataset_generator::copy_dataset_to_new_folder]')
    print('\t start copy dataset to new folder...')
    copied_mesh_num = 0
    for mesh_filename in tqdm(source_mesh_filename_list):
        if copied_mesh_num >= copy_file_num:
            break

        save_mesh_file_path = save_mesh_folder_path + mesh_filename

        mesh_file_basename = mesh_filename.split('.')[0]
        if mesh_file_basename in skip_id_list:
            removeFile(save_mesh_file_path)
            continue

        if os.path.exists(save_mesh_file_path):
            copied_mesh_num += 1
            continue

        source_mesh_file_path = source_mesh_folder_path + mesh_filename

        copyfile(source_mesh_file_path, save_mesh_file_path)
        copied_mesh_num += 1

    return True

def demo():
    dataset_root_folder_path = '/home/chli/chLi/Dataset/'
    sample_point_num_list = [10000, 100000]
    random_strength_list = [0.005, 0.01, 0.02]
    gauss_mean_list = [0.0]
    gauss_sigma_list = [0.005, 0.01, 0.02]
    impulse_strength_list = [0.02, 0.04, 0.08]
    impulse_probability_list = [0.1]
    overwrite = False

    need_copy_dataset_to_new_folder = True
    new_dataset_root_folder_path = '/home/chli/Dataset/'
    copy_file_num = 100

    skip_id_list = ['157936971ef9b6bb858b20d410ebdb99',]

    noise_dict_list = []
    for sample_point_num in sample_point_num_list:
        for random_strength in random_strength_list:
            random_noise_dict = {'sample_point_num': sample_point_num, 'type': 'Random', 'strength': random_strength}
            noise_dict_list.append(random_noise_dict)

        for gauss_mean in gauss_mean_list:
            for gauss_sigma in gauss_sigma_list:
                gauss_noise_dict = {'sample_point_num': sample_point_num, 'type': 'Gauss', 'mean': gauss_mean, 'sigma': gauss_sigma}
                noise_dict_list.append(gauss_noise_dict)

        for impulse_strength in impulse_strength_list:
            for impulse_probability in impulse_probability_list:

                impulse_noise_dict = {'sample_point_num': sample_point_num, 'type': 'Impulse', 'strength': impulse_strength, 'probability': impulse_probability}
                noise_dict_list.append(impulse_noise_dict)

    if need_copy_dataset_to_new_folder:
        copy_dataset_to_new_folder(dataset_root_folder_path, new_dataset_root_folder_path, copy_file_num, skip_id_list)
        dataset_root_folder_path = new_dataset_root_folder_path

    noise_dataset_generator = NoiseDatasetGenerator(dataset_root_folder_path)
    noise_dataset_generator.autoGenerateNoisePcds(noise_dict_list, overwrite, skip_id_list)
    return True
