from open3d_manage.Method.io import toPLY
from open3d_manage.Method.render import renderGeometries


def demo():
    mesh_file_path = "/Users/fufu/Downloads/Airplane without texture.stl/Airplane without texture.stl"

    mesh = toPLY(mesh_file_path)
    renderGeometries(mesh)
    return True
