import open3d as o3d
from open3d_manage.Module.noise_adder import NoiseAdder

def demo():
    mesh_file_path = "/home/chli/chLi/Dataset/ManifoldMesh/ShapeNet/02691156/2af04ef09d49221b85e5214b0d6a7.obj"
    sample_point_num = 10000
    random_strength = 1.0
    gauss_mean = 0.0
    gauss_sigma = 1.0
    impulse_strength = 1.0
    impulse_probability = 0.1
    save_random_noise_pcd_file_path = "/home/chli/chLi/Dataset/ManifoldMesh_NoisePcd/ShapeNet/02691156/2af04ef09d49221b85e5214b0d6a7/Random/strength-" + str(random_strength) + ".ply"
    save_gauss_noise_pcd_file_path = "/home/chli/chLi/Dataset/ManifoldMesh_NoisePcd/ShapeNet/02691156/2af04ef09d49221b85e5214b0d6a7/Gauss/mean-" + str(gauss_mean) + "_sigma-" + str(gauss_sigma) + ".ply"
    save_impulse_noise_pcd_file_path = "/home/chli/chLi/Dataset/ManifoldMesh_NoisePcd/ShapeNet/02691156/2af04ef09d49221b85e5214b0d6a7/Impulse/probility-" + str(impulse_probability) + ".ply"
    overwrite = False
    need_reset = True

    mesh = o3d.io.read_triangle_mesh(mesh_file_path)
    pcd = mesh.sample_points_poisson_disk(sample_point_num)

    noise_adder = NoiseAdder(pcd)
    noise_adder.addRandomNoise(random_strength)
    noise_adder.save(save_random_noise_pcd_file_path, overwrite, need_reset)
    noise_adder.addGaussNoise(gauss_mean, gauss_sigma)
    noise_adder.save(save_gauss_noise_pcd_file_path, overwrite, need_reset)
    noise_adder.addImpulseNoise(impulse_strength, impulse_probability)
    noise_adder.save(save_impulse_noise_pcd_file_path, overwrite, need_reset)
    return True
