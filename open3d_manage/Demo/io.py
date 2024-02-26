from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.render import renderGeometries


def demo():
    geometry_file_path = "/Users/fufu/Downloads/Airplane without texture.stl/Airplane without texture.stl"
    print_progress = True

    renderGeometries(
        loadGeometry(geometry_file_path, "mesh", print_progress), "airplane"
    )
    return True
