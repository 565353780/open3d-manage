from open3d_manage.Module.o3d_viewer import O3DViewer


def demo():
    obj_file_path = "/Users/fufu/Downloads/bunny.obj"

    o3d_viewer = O3DViewer()
    o3d_viewer.createWindow()

    o3d_viewer.loadMeshFile(obj_file_path)
    for _ in range(100):
        o3d_viewer.rotateView(0, 10)
    o3d_viewer.run()

    return True
