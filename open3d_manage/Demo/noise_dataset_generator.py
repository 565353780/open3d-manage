from open3d_manage.Module.noise_dataset_generator import NoiseDatasetGenerator

def demo():
    dataset_root_folder_path = '/home/chli/chLi/Dataset/'
    noise_dict_list = [
        {'sample_point_num': 10000, 'type': 'Random', 'strength': 1.0},
        {'sample_point_num': 10000, 'type': 'Random', 'strength': 2.0},
        {'sample_point_num': 10000, 'type': 'Random', 'strength': 4.0},

        {'sample_point_num': 10000, 'type': 'Gauss', 'mean': 0.0, 'sigma': 1.0},
        {'sample_point_num': 10000, 'type': 'Gauss', 'mean': 0.0, 'sigma': 2.0},
        {'sample_point_num': 10000, 'type': 'Gauss', 'mean': 0.0, 'sigma': 4.0},

        {'sample_point_num': 10000, 'type': 'Impulse', 'strength': 1.0, 'probability': 0.1},
        {'sample_point_num': 10000, 'type': 'Impulse', 'strength': 2.0, 'probability': 0.1},
        {'sample_point_num': 10000, 'type': 'Impulse', 'strength': 4.0, 'probability': 0.1},

        {'sample_point_num': 100000, 'type': 'Random', 'strength': 1.0},
        {'sample_point_num': 100000, 'type': 'Random', 'strength': 2.0},
        {'sample_point_num': 100000, 'type': 'Random', 'strength': 4.0},

        {'sample_point_num': 100000, 'type': 'Gauss', 'mean': 0.0, 'sigma': 1.0},
        {'sample_point_num': 100000, 'type': 'Gauss', 'mean': 0.0, 'sigma': 2.0},
        {'sample_point_num': 100000, 'type': 'Gauss', 'mean': 0.0, 'sigma': 4.0},

        {'sample_point_num': 100000, 'type': 'Impulse', 'strength': 1.0, 'probability': 0.1},
        {'sample_point_num': 100000, 'type': 'Impulse', 'strength': 2.0, 'probability': 0.1},
        {'sample_point_num': 100000, 'type': 'Impulse', 'strength': 4.0, 'probability': 0.1},
    ]
    overwrite = False

    noise_dataset_generator = NoiseDatasetGenerator(dataset_root_folder_path)
    noise_dataset_generator.autoGenerateNoisePcds(noise_dict_list, overwrite)
    return True
