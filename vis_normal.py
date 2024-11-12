import numpy as np
import open3d as o3d

pcd_file_path = '/Users/fufu/Downloads/Dataset/AMCAX/PGR/base-plane_gauss_2w_sample-20000_k_7_min_0.0015_max_0.015_alpha_1.8_depth_min_1_depth_max_1_lse.xyz'

points = []
normals = []

with open(pcd_file_path, 'r') as f:
    for line in f.readlines():
        data_list = line.split()
        points.append([float(data_list[i]) for i in range(3)])
        normals.append([float(data_list[i]) for i in range(3, 6)])
points = np.array(points)
normals = np.array(normals)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
pcd.normals = o3d.utility.Vector3dVector(normals)
pcd.normalize_normals()

points = np.asarray(pcd.points)
normals = np.asarray(pcd.normals)

arrow_length = 0.01

line_set = o3d.geometry.LineSet()
line_points = []
lines = []

# 构造点和线
for i in range(len(points)):
    start_point = points[i]
    end_point = start_point + normals[i] * arrow_length
    line_points.append(start_point)
    line_points.append(end_point)
    lines.append([i * 2, i * 2 + 1])  # 每条线连接两个点

# 设置 LineSet 的点和线属性
all_points = np.array(line_points)  # 所有起点和终点
lines = np.array(lines, dtype=np.int32)
line_set.points = o3d.utility.Vector3dVector(all_points)
line_set.lines = o3d.utility.Vector2iVector(lines)
# line_set.paint_uniform_color(np.array([[1.0], [0.0], [0.0]], dtype=np.float64))  # 将法向量线设置为红色

o3d.visualization.draw_geometries([pcd, line_set], point_show_normal=False)
