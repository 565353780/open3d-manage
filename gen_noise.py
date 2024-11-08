import open3d as o3d
from open3d_manage.Module.mesh_noise_adder import MeshNoiseAdder

if __name__ == "__main__":
    mesh_file_path = "/home/chli/github/AMCAX/point-cloud-reverse-algorithm/data/testdata-20241105/base-plane.ply"
    sample_point_num = 20000
    gauss_mean = 0.0
    # 1mm / 36m
    noise_scale = 1.0 / 36.0 / 1000.0
    save_gauss_noise_pcd_file_path = "/home/chli/github/AMCAX/point-cloud-reverse-algorithm/data/testdata-20241105/base-plane_gauss_2w.ply"
    overwrite = False

    mesh = o3d.io.read_triangle_mesh(mesh_file_path)
    aabb = mesh.get_axis_aligned_bounding_box()
    max_length = max(aabb.get_extent())
    gauss_sigma = max_length * noise_scale
    print('gauss_sigma:', gauss_sigma)

    mesh_noise_adder = MeshNoiseAdder(mesh)
    mesh_noise_adder.addGaussNoise(sample_point_num, gauss_mean, gauss_sigma)
    mesh_noise_adder.save(save_gauss_noise_pcd_file_path, overwrite)
