import os

from open3d_manage.Method.io import loadGeometry, saveGeometry
from open3d_manage.Method.noise import toGaussNoisePCD


def toPLYFile(
    geometry_file_path: str,
    geometry_type: str,
    save_ply_file_path: str,
    overwrite: bool = False,
    print_progress: bool = False,
) -> bool:
    if not overwrite:
        if os.path.exists(save_ply_file_path):
            print("[ERROR][trans::toPLYFile]")
            print("\t save ply file already exist!")
            print("\t save_ply_file_path:", save_ply_file_path)
            return False

    if save_ply_file_path[-4:] != ".ply":
        print("[ERROR][trans::toPLYFile]")
        print("\t save ply file have wrong extension!")
        print("\t save_ply_file_path:", save_ply_file_path)
        return False

    geometry = loadGeometry(geometry_file_path, geometry_type, print_progress)

    if geometry is None:
        print("[ERROR][trans::toPLYFile]")
        print("\t loadGeometry failed!")
        return False

    if not saveGeometry(save_ply_file_path, geometry, overwrite, print_progress):
        print("[ERROR][trans::toPLYFile]")
        print("\t saveGeometry failed!")
        return False

    return True


def toPCDFile(
    geometry_file_path: str,
    geometry_type: str,
    sample_point_num: int,
    save_pcd_file_path: str,
    overwrite: bool = False,
    print_progress: bool = False,
) -> bool:
    if not overwrite:
        if os.path.exists(save_pcd_file_path):
            print("[ERROR][trans::toPCDFile]")
            print("\t save pcd file already exist!")
            print("\t save_pcd_file_path:", save_pcd_file_path)
            return False

    if save_pcd_file_path[-4:] != ".pcd":
        print("[ERROR][trans::toPCDFile]")
        print("\t save pcd file have wrong extension!")
        print("\t save_pcd_file_path:", save_pcd_file_path)
        return False

    geometry = loadGeometry(geometry_file_path, geometry_type, print_progress)

    if geometry is None:
        print("[ERROR][trans::toPCDFile]")
        print("\t loadGeometry failed!")
        return False

    match geometry_type:
        case "pcd":
            pcd = geometry
        case "mesh":
            pcd = geometry.sample_points_poisson_disk(sample_point_num)
        case _:
            print("[ERROR][trans::toPCDFile]")
            print("\t geometry type to pcd function not defined!")
            print("\t geometry_type:", geometry_type)
            return False

    if not saveGeometry(save_pcd_file_path, pcd, overwrite, print_progress):
        print("[ERROR][trans::toPCDFile]")
        print("\t saveGeometry failed!")
        return False

    return True


def toGaussNoisePCDFile(
    pcd_file_path: str,
    gauss_mean: float,
    gauss_sigma: float,
    save_pcd_file_path: str,
    overwrite: bool = False,
    print_progress: bool = False,
) -> bool:
    if not overwrite:
        if os.path.exists(save_pcd_file_path):
            print("[ERROR][trans::toGaussNoisePCDFile]")
            print("\t save pcd file already exist!")
            print("\t save_pcd_file_path:", save_pcd_file_path)
            return False

    if save_pcd_file_path[-4:] != ".pcd":
        print("[ERROR][trans::toGaussNoisePCDFile]")
        print("\t save pcd file have wrong extension!")
        print("\t save_pcd_file_path:", save_pcd_file_path)
        return False

    pcd = loadGeometry(pcd_file_path, "pcd", print_progress)

    if pcd is None:
        print("[ERROR][trans::toGaussNoisePCDFile]")
        print("\t loadGeometry failed!")
        return False

    gauss_noise_pcd = toGaussNoisePCD(pcd, gauss_mean, gauss_sigma)

    if not saveGeometry(save_pcd_file_path, gauss_noise_pcd, overwrite, print_progress):
        print("[ERROR][trans::toGaussNoisePCDFile]")
        print("\t saveGeometry failed!")
        return False

    return True
