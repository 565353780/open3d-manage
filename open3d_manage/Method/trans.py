import os

from open3d_manage.Method.io import loadGeometry, saveGeometry
from open3d_manage.Method.path import removeFile


def toPLY(
    geometry_file_path: str,
    geometry_type: str,
    save_ply_file_path: str,
    overwrite: bool = False,
    print_progress: bool = False,
) -> bool:
    if os.path.exists(save_ply_file_path):
        if overwrite:
            removeFile(save_ply_file_path)
        else:
            print("[ERROR][trans::toPLY]")
            print("\t save ply file already exist!")
            print("\t save_ply_file_path:", save_ply_file_path)
            return False

    if save_ply_file_path[-4:] != ".ply":
        print("[ERROR][trans::toPLY]")
        print("\t save ply file have wrong extension!")
        print("\t save_ply_file_path:", save_ply_file_path)
        return False

    geometry = loadGeometry(geometry_file_path, geometry_type, print_progress)

    if geometry is None:
        print("[ERROR][trans::toPLY]")
        print("\t loadGeometry failed!")
        return False

    if not saveGeometry(save_ply_file_path, geometry, print_progress):
        print("[ERROR][trans::toPLY]")
        print("\t saveGeometry failed!")
        return False

    return True


def toPCD(
    geometry_file_path: str,
    geometry_type: str,
    sample_point_num: int,
    save_pcd_file_path: str,
    overwrite: bool = False,
    print_progress: bool = False,
) -> bool:
    if os.path.exists(save_pcd_file_path):
        if overwrite:
            removeFile(save_pcd_file_path)
        else:
            print("[ERROR][trans::toPCD]")
            print("\t save pcd file already exist!")
            print("\t save_pcd_file_path:", save_pcd_file_path)
            return False

    if save_pcd_file_path[-4:] != ".pcd":
        print("[ERROR][trans::toPCD]")
        print("\t save pcd file have wrong extension!")
        print("\t save_pcd_file_path:", save_pcd_file_path)
        return False

    geometry = loadGeometry(geometry_file_path, geometry_type, print_progress)

    if geometry is None:
        print("[ERROR][trans::toPCD]")
        print("\t loadGeometry failed!")
        return False

    match geometry_type:
        case "pcd":
            pcd = geometry
        case "mesh":
            pcd = geometry.sample_points_poisson_disk(sample_point_num)
        case _:
            print("[ERROR][trans::toPCD]")
            print("\t geometry type to pcd function not defined!")
            print("\t geometry_type:", geometry_type)
            return False

    if not saveGeometry(save_pcd_file_path, pcd, print_progress):
        print("[ERROR][trans::toPCD]")
        print("\t saveGeometry failed!")
        return False

    return True
