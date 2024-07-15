import os
import numpy as np
import open3d as o3d

mesh_file_path = "path/to/your/mesh/file.obj"
sample_point_num = 10000
save_pcd_file_path = "./output/sampled_point_cloud.ply"

# 读取网格文件
mesh = o3d.io.read_triangle_mesh(mesh_file_path)  # 替换为你的文件路径

# 计算法线（如果网格没有法线）
if not mesh.has_vertex_normals():
    mesh.compute_vertex_normals()

# 从网格中均匀采样10000个点
point_cloud = mesh.sample_points_uniformly(number_of_points=sample_point_num)

# 获取采样的点，作为np.ndarray数组
points = np.asarray(point_cloud.points)
print(points.shape)

# 可视化采样点云
o3d.visualization.draw_geometries([point_cloud])

# 如果你想将点云保存到文件
os.makedirs('./output/')
o3d.io.write_point_cloud(save_pcd_file_path, point_cloud)
