from open3d_manage.Method.io import loadMesh
from open3d_manage.Method.trans import toPLY
from open3d_manage.Method.render import renderGeometries


def demo():
    mesh_file_path = "/Users/fufu/Downloads/Airplane without texture.stl/Airplane without texture.stl"
    ply_file_path = "./output/airplane.ply"
    overwrite = False
    print_progress = True

    toPLY(mesh_file_path, ply_file_path, overwrite, print_progress)
    renderGeometries(loadMesh(ply_file_path), "airplane")
    return True
