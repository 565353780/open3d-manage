from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.trans import toPLYFile, toPCDFile, toGaussNoisePCDFile
from open3d_manage.Method.render import renderGeometries


def demo():
    geometry_file_path = "/Users/fufu/Downloads/Airplane without texture.stl/Airplane without texture.stl"
    geometry_type = "mesh"
    ply_file_path = "./output/airplane.ply"

    sample_point_num = 1000000
    pcd_file_path = "./output/airplane_pcd.ply"

    gauss_mean = 100.0
    gauss_sigma = 100.0
    gauss_noise_pcd_file_path = (
        "./output/airplane"
        + "_gaussMean-" str(gauss_mean)
        + "_gaussSigma-" + str(gauss_sigma)
        + "_pcd.ply"
    )

    overwrite = False
    print_progress = True

    toPLYFile(
        geometry_file_path, geometry_type, ply_file_path, overwrite, print_progress
    )

    toPCDFile(
        ply_file_path,
        geometry_type,
        sample_point_num,
        pcd_file_path,
        overwrite,
        print_progress,
    )

    if False:
        toGaussNoisePCDFile(
            pcd_file_path,
            gauss_mean,
            gauss_sigma,
            gauss_noise_pcd_file_path,
            overwrite,
            print_progress,
        )

        gauss_noise_pcd = loadGeometry(
            gauss_noise_pcd_file_path, "pcd", print_progress)

        renderGeometries(gauss_noise_pcd, "gauss noise airplane")

    if True:
        for noise_params in [[0.1, 0.1], [1.0, 1.0], [10.0, 10.0], [100.0, 100.0]]:
            gauss_mean, gauss_sigma = noise_params

            gauss_noise_pcd_file_path = (
                "./output/airplane"
                + "_gaussMean-" str(gauss_mean)
                + "_gaussSigma-" + str(gauss_sigma)
                + "_pcd.ply"
            )

            toGaussNoisePCDFile(
                pcd_file_path,
                gauss_mean,
                gauss_sigma,
                gauss_noise_pcd_file_path,
                overwrite,
                print_progress,
            )
    return True
