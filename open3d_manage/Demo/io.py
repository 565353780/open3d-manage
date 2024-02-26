from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.trans import toPLY
from open3d_manage.Method.render import renderGeometries


def demo():
    geometry_file_path = "/Users/fufu/Downloads/Airplane without texture.stl/Airplane without texture.stl"
    geometry_type = "mesh"
    ply_file_path = "./output/airplane.ply"
    overwrite = False
    print_progress = True

    toPLY(geometry_file_path, geometry_type, ply_file_path, overwrite, print_progress)

    renderGeometries(loadGeometry(ply_file_path, "mesh", print_progress), "airplane")
    return True
