import os
import numpy as np
import open3d as o3d
from typing import Union

from open3d_manage.Method.path import createFileFolder, removeFile


class O3DViewer(object):
    def __init__(self) -> None:
        self.vis = o3d.visualization.Visualizer()
        return

    def createWindow(
        self,
        window_name: str = "Open3D",
        width: int = 1920,
        height: int = 1080,
        left: int = 50,
        top: int = 50,
        visible: bool = True,
    ) -> bool:
        self.vis.create_window(window_name, width, height, left, top, visible)
        return True

    def rotateView(self, x: float, y: float) -> bool:
        ctr = self.vis.get_view_control()
        ctr.rotate(x, y)
        self.update()
        return True

    def addGeometry(
        self, geometry: o3d.geometry.Geometry, reset_bbox: bool = True
    ) -> bool:
        self.vis.add_geometry(geometry, reset_bbox)
        return True

    def addGeometries(self, geometry_list: list, reset_bbox: bool = True) -> bool:
        for geometry in geometry_list:
            self.vis.add_geometry(geometry, reset_bbox)
        return True

    def loadMeshFile(
        self, mesh_file_path: str, reset_bbox: bool = True, print_progress: bool = False
    ) -> bool:
        if not os.path.exists(mesh_file_path):
            print("[ERROR][O3DViewer::loadMeshFile]")
            print("\t mesh file not exist!")
            print("\t mesh_file_path:", mesh_file_path)
            return False

        mesh = o3d.io.read_triangle_mesh(mesh_file_path, print_progress=print_progress)
        mesh.compute_vertex_normals()
        mesh.compute_triangle_normals()

        self.addGeometry(mesh, reset_bbox)
        return True

    def loadPcdFile(
        self, pcd_file_path: str, reset_bbox: bool = True, print_progress: bool = False
    ) -> bool:
        if not os.path.exists(pcd_file_path):
            print("[ERROR][O3DViewer::loadPcdFile]")
            print("\t pcd file not exist!")
            print("\t pcd_file_path:", pcd_file_path)
            return False

        pcd = o3d.io.read_point_cloud(pcd_file_path, print_progress=print_progress)
        pcd.estimate_normals()
        pcd.orient_normals_consistent_tangent_plane(100)

        self.addGeometry(pcd, reset_bbox)
        return True

    def removeGeometry(
        self, geometry: o3d.geometry.Geometry, reset_bbox: bool = True
    ) -> bool:
        self.vis.remove_geometry(geometry, reset_bbox)
        return True

    def removeGeometries(self, geometry_list: list, reset_bbox: bool = True) -> bool:
        for geometry in geometry_list:
            self.vis.remove_geometry(geometry, reset_bbox)
        return True

    def replaceGeometry(
        self,
        old_geometry: o3d.geometry.Geometry,
        new_geometry: o3d.geometry.Geometry,
        reset_bbox: bool = False,
    ) -> bool:
        self.removeGeometry(old_geometry)
        self.addGeometry(new_geometry)
        return True

    def updateGeometry(self, geometry: o3d.geometry.Geometry) -> bool:
        self.vis.update_geometry(geometry)
        return True

    def clearGeometries(self, reset_view_point: bool = True) -> bool:
        self.vis.clear_geometries()
        self.vis.reset_view_point(reset_view_point)
        return True

    def addLabel(self, position: Union[list, np.ndarray], label: str) -> bool:
        self.vis.add_3d_label(np.ndarray(position, dtype=float), label)
        return True

    def addLabels(self, positions: Union[list, np.ndarray], labels: list) -> bool:
        for position, label in zip(positions, labels):
            self.addLabel(position, label)
        return True

    def update(self) -> bool:
        self.vis.poll_events()
        self.vis.update_renderer()
        return True

    def captureScreenImage(self, image_file_path: str, overwrite: bool = False) -> bool:
        if os.path.exists(image_file_path):
            if overwrite:
                removeFile(image_file_path)
            else:
                print("[WARN][O3DViewer::captureScreenImage]")
                print("\t image file already exist! please choose another save path!")
                print("\t image_file_path:", image_file_path)
                return True

        createFileFolder(image_file_path)

        self.vis.capture_screen_image(image_file_path)
        return True

    def run(self) -> bool:
        self.vis.run()
        return True

    def destoryWindow(self) -> bool:
        self.vis.destroy_window()
        return True
