import numpy as np
import open3d as o3d
import plotly.graph_objs as go
from copy import deepcopy


def renderGeometries(geometries, window_name: str = "Open3D Renderer") -> bool:
    if not isinstance(geometries, list):
        geometries = [geometries]

    o3d.visualization.draw_geometries(geometries, window_name=window_name)
    return True


def toFilterWeights(curvatures):
    # for i in range(30):
    #     std = np.std(curvatures)
    #     mean = np.mean(curvatures)
    #     print(f"std:{std}, mean:{mean}")

    #     dlimit = mean - 3 * std
    #     ulimit = mean + 3 * std

    #     curvatures = np.where(curvatures < dlimit, 0, curvatures)
    #     curvatures = np.where(curvatures > ulimit, ulimit, curvatures)
    # mean = np.mean(curvatures)
    # weights = np.ones_like(curvatures)
    # weights = np.where(curvatures >= mean, np.exp(-(curvatures-mean)**2/mean**2), weights)

    # -----------------箱线剔除离群点--------------------------
    Q1 = np.percentile(curvatures, 25)
    Q3 = np.percentile(curvatures, 75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    curvatures = np.where(curvatures < lower_bound, 0, curvatures)
    curvatures = np.where(curvatures > upper_bound, upper_bound, curvatures)
    mean = np.mean(curvatures)
    weights = np.ones_like(curvatures)
    weights = np.where(
        curvatures >= mean, np.exp(-((curvatures - mean) ** 2) / mean**2), weights
    )

    std = np.std(curvatures)
    print(f"std:{std}, mean:{mean}")
    return weights


def visualize_curvature(pcd: o3d.geometry.PointCloud, curvatures: np.ndarray) -> bool:
    """by Junyi Liu"""
    render_pcd = deepcopy(pcd)

    point_num = np.asarray(render_pcd.points).shape[0]

    abs_curvatures = np.abs(curvatures)

    weights = toFilterWeights(abs_curvatures)

    # min = np.min(weights)
    # max = np.max(weights)

    curvature_colors = np.zeros((point_num, 3))

    for point_idx in range(point_num):
        # colormap
        # weight = weights[point_idx]
        # ratio = (weight - min) / (max - min)
        ratio = 1 - weights[point_idx]

        if 0 <= ratio < 0.25:
            curvature_colors[point_idx] = [0, ratio * 4, 1]
        elif ratio < 0.5:
            curvature_colors[point_idx] = [0, 1, 1 - (ratio - 0.25) * 4]
        elif ratio < 0.75:
            curvature_colors[point_idx] = [(ratio - 0.5) * 4, 1, 0]
        elif ratio < 1:
            curvature_colors[point_idx] = [1, 1 - (ratio - 0.75) * 4, 0]
        else:
            curvature_colors[point_idx] = [1, 0, 0]

    render_pcd.colors = o3d.utility.Vector3dVector(curvature_colors)

    renderGeometries(render_pcd, "curvature")
    o3d.io.write_point_cloud(
        "D:\\Program Files\\dev_for_python\\data\\result\\fit_curvature_knn=15.pcd",
        render_pcd,
    )
    return True


def toPlotFigure(points: np.ndarray):
    trace = go.Scatter3d(
        x=points[:, 0],
        y=points[:, 1],
        z=points[:, 2],
        mode="markers",
        marker=dict(size=2),
    )

    layout = go.Layout(
        scene=dict(
            xaxis=dict(
                title="",
                showgrid=False,
                zeroline=False,
                showline=False,
                ticks="",
                showticklabels=False,
            ),
            yaxis=dict(
                title="",
                showgrid=False,
                zeroline=False,
                showline=False,
                ticks="",
                showticklabels=False,
            ),
            zaxis=dict(
                title="",
                showgrid=False,
                zeroline=False,
                showline=False,
                ticks="",
                showticklabels=False,
            ),
        ),
        margin=dict(l=0, r=0, b=0, t=0),
        showlegend=False,
    )

    return go.Figure(data=[trace], layout=layout)
