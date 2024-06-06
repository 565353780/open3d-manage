import numpy as np
import open3d as o3d
import plotly.graph_objs as go
from copy import deepcopy

from open3d_manage.Method.filter import toFilterWeights

def renderGeometries(geometries, window_name: str = "Open3D Renderer") -> bool:
    if not isinstance(geometries, list):
        geometries = [geometries]

    o3d.visualization.draw_geometries(geometries, window_name=window_name)
    return True


def visualize_curvature(pcd: o3d.geometry.PointCloud, curvatures: np.ndarray) -> bool:
    """by Junyi Liu"""
    render_pcd = deepcopy(pcd)

    point_num = np.asarray(render_pcd.points).shape[0]

    weights = toFilterWeights(np.abs(curvatures))

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
