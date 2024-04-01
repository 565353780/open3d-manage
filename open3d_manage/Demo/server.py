from open3d_manage.Module.server import Server


def demo():
    port = 6003

    server = Server(port)

    server.start()
    return True
