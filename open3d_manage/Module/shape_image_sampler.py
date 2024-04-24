import os
import numpy as np
import open3d as o3d
from copy import deepcopy


from open3d_manage.Method.path import createFileFolder
from open3d_manage.Module.o3d_viewer import O3DViewer


class ShapeImageSampler(object):
    def __init__(
        self,
        window_name: str = "Open3D",
        width: int = 1920,
        height: int = 1080,
        left: int = 50,
        top: int = 50,
        visible: bool = True,
    ) -> None:
        self.o3d_viewer = O3DViewer()
        self.o3d_viewer.createWindow(window_name, width, height, left, top, visible)
        return

    def sampleImages(
        self,
        shape_file_path: str,
        save_folder_path: str,
        y_rotate_num: int = 8,
        x_rotate_num: int = 5,
        overwrite: bool = False,
    ) -> bool:
        if not os.path.exists(shape_file_path):
            print("[ERROR][ShapeImageSampler::sampleImages]")
            print("\t shape file not exist!")
            print("\t shape_file_path:", shape_file_path)
            return False

        mesh = o3d.io.read_triangle_mesh(shape_file_path)
        mesh.compute_vertex_normals()
        mesh.compute_triangle_normals()

        for i in range(y_rotate_num):
            y_rad = 2.0 * np.pi * i / y_rotate_num
            cos_y = np.cos(y_rad)
            sin_y = np.sin(y_rad)

            y_rotate_matrix = np.array(
                [
                    [cos_y, 0, -sin_y],
                    [0, 1, 0],
                    [sin_y, 0, cos_y],
                ],
                dtype=float,
            )

            for j in range(x_rotate_num):
                x_rad = np.pi * (j / (x_rotate_num - 1) - 0.5)
                cos_x = np.cos(x_rad)
                sin_x = np.sin(x_rad)

                x_rotate_matrix = np.array(
                    [
                        [1, 0, 0],
                        [0, cos_x, -sin_x],
                        [0, sin_x, cos_x],
                    ],
                    dtype=float,
                )

                rotate_matrix = x_rotate_matrix.dot(y_rotate_matrix)

                rotate_mesh = deepcopy(mesh)

                rotate_mesh.rotate(rotate_matrix)

                self.o3d_viewer.clearGeometries()
                self.o3d_viewer.addGeometry(rotate_mesh)
                self.o3d_viewer.update()

                save_file_path = (
                    save_folder_path + "y_" + str(i) + "_x_" + str(j) + ".png"
                )

                createFileFolder(save_file_path)

                self.o3d_viewer.captureScreenImage(save_file_path, overwrite)
        return True
