import os
import open3d as o3d

from open3d_manage.Method.io import loadMesh, saveMesh
from open3d_manage.Method.path import removeFile


def toPLY(
    mesh_file_path: str,
    save_ply_file_path: str,
    overwrite: bool = False,
    print_progress: bool = False,
) -> o3d.geometry.TriangleMesh:
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

    mesh = loadMesh(mesh_file_path, print_progress)

    if mesh is None:
        print("[ERROR][trans::toPLY]")
        print("\t loadMesh failed!")
        return False

    if not saveMesh(save_ply_file_path, mesh, print_progress):
        print("[ERROR][trans::toPLY]")
        print("\t saveMesh failed!")
        return False

    return True
