from open3d_manage.Method.io import loadGeometry
from open3d_manage.Method.trans import toPLY, toPCD
from open3d_manage.Method.render import renderGeometries


def demo():
    geometry_file_path = "/Users/fufu/Downloads/Airplane without texture.stl/Airplane without texture.stl"
    geometry_type = "mesh"
    ply_file_path = "./output/airplane.ply"
    pcd_file_path = "./output/airplane.pcd"
    sample_point_num = 1000000
    overwrite = False
    print_progress = True

    toPLY(geometry_file_path, geometry_type,
          ply_file_path, overwrite, print_progress)

    toPCD(
        ply_file_path,
        geometry_type,
        sample_point_num,
        pcd_file_path,
        overwrite,
        print_progress,
    )

    renderGeometries(loadGeometry(
        pcd_file_path, "pcd", print_progress), "airplane")
    return True
