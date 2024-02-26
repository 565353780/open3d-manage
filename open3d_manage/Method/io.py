import os
import open3d as o3d
from typing import Union

from open3d_manage.Method.path import createFileFolder, renameFile


def loadMesh(
    mesh_file_path: str,
    print_progress: bool = False,
) -> Union[o3d.geometry.TriangleMesh, None]:
    if not os.path.exists(mesh_file_path):
        print("[ERROR][io::loadMesh]")
        print("\t mesh file not exist!")
        print("\t mesh_file_path:", mesh_file_path)
        return None

    return o3d.io.read_triangle_mesh(mesh_file_path, print_progress=print_progress)


def saveMesh(
    save_mesh_file_path: str,
    mesh: o3d.geometry.TriangleMesh,
    print_progress: bool = False,
) -> bool:
    createFileFolder(save_mesh_file_path)

    tmp_save_mesh_file_path = save_mesh_file_path[:-4] + "_tmp.ply"

    o3d.io.write_triangle_mesh(
        tmp_save_mesh_file_path, mesh, write_ascii=True, print_progress=print_progress
    )

    if not os.path.exists(tmp_save_mesh_file_path):
        print("[ERROR][io::saveMesh]")
        print("\t write_triangle_mesh failed!")
        print("\t save_mesh_file_path:", save_mesh_file_path)
        return False

    renameFile(tmp_save_mesh_file_path, save_mesh_file_path)

    return True
