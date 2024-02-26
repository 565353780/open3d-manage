import os
import open3d as o3d
from typing import Union

from open3d_manage.Config.type import VALID_TYPES
from open3d_manage.Method.path import createFileFolder, renameFile


def loadGeometry(
    geometry_file_path: str,
    geometry_type: str,
    print_progress: bool = False,
) -> Union[o3d.geometry.TriangleMesh, o3d.geometry.PointCloud, None]:
    if not os.path.exists(geometry_file_path):
        print("[ERROR][io::loadGeometry]")
        print("\t " + geometry_type + " file not exist!")
        print("\t geometry_file_path:", geometry_file_path)
        return None

    match geometry_type:
        case "mesh":
            return o3d.io.read_triangle_mesh(
                geometry_file_path, print_progress=print_progress
            )
        case "pcd":
            return o3d.io.read_point_cloud(
                geometry_file_path, print_progress=print_progress
            )
        case _:
            print("[ERROR][io::loadGeometry]")
            print("\t geometry type not valid!")
            print("\t geometry_type:", geometry_type)
            print("\t VALID_TYPES:", VALID_TYPES)
            return None


def saveGeometry(
    save_geometry_file_path: str,
    geometry: Union[o3d.geometry.TriangleMesh, o3d.geometry.PointCloud],
    print_progress: bool = False,
) -> bool:
    createFileFolder(save_geometry_file_path)

    tmp_save_geometry_file_path = (
        save_geometry_file_path[:-4] + "_tmp" + save_geometry_file_path[-4:]
    )

    if isinstance(geometry, o3d.geometry.TriangleMesh):
        o3d.io.write_triangle_mesh(
            tmp_save_geometry_file_path,
            geometry,
            write_ascii=True,
            print_progress=print_progress,
        )
    elif isinstance(geometry, o3d.geometry.PointCloud):
        o3d.io.write_point_cloud(
            tmp_save_geometry_file_path,
            geometry,
            write_ascii=True,
            print_progress=print_progress,
        )

    if not os.path.exists(tmp_save_geometry_file_path):
        print("[ERROR][io::saveGeometry]")
        if isinstance(geometry, o3d.geometry.TriangleMesh):
            print("\t write_triangle_mesh failed!")
        elif isinstance(geometry, o3d.geometry.PointCloud):
            print("\t write_point_cloud failed!")
        else:
            print("\t write function for geometry not defined!")
        print("\t save_geometry_file_path:", save_geometry_file_path)
        return False

    renameFile(tmp_save_geometry_file_path, save_geometry_file_path)

    return True
