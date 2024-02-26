import open3d as o3d


def toPLY(mesh_file_path: str) -> o3d.geometry.TriangleMesh:
    return o3d.io.read_triangle_mesh(mesh_file_path)
