import open3d as o3d


def renderGeometries(geometries, window_name: str = "Open3D Renderer") -> bool:
    if not isinstance(geometries, list):
        geometries = [geometries]

    o3d.visualization.draw_geometries(geometries, window_name=window_name)
    return True
