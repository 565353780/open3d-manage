import os
import numpy as np
import gradio as gr
import open3d as o3d

from open3d_manage.Method.io import loadGeometry, saveGeometry
from open3d_manage.Method.curvature import estimateCurvaturesByFit
from open3d_manage.Method.filter import bilateral_filter, toFilterWeights
from open3d_manage.Method.render import toPlotFigure


def renderInputData(input_pcd_file_path: str):
    pcd = o3d.io.read_point_cloud(input_pcd_file_path)

    gt_points = np.asarray(pcd.points)

    return toPlotFigure(gt_points)


def toDenoisedPcd(
    input_pcd_file_path: str,
    sigma_d: float,
    sigma_n: int,
    curvature_knn_num: int,
    filter_knn_num: int,
):
    print("input_pcd_file_path:", input_pcd_file_path)
    if not os.path.exists(input_pcd_file_path):
        print("[ERROR][Server::fitBSplineSurface]")
        print("\t input pcd file not exist!")
        print("\t input_pcd_file_path:", input_pcd_file_path)
        return ""

    input_pcd_file_name = input_pcd_file_path.split("/")[-1]
    save_pcd_file_path = "./output/" + input_pcd_file_name
    overwrite = True
    print_progress = True

    noise_pcd = loadGeometry(
        input_pcd_file_path,
        "pcd",
        print_progress,
    )

    curvatures = estimateCurvaturesByFit(noise_pcd, curvature_knn_num, print_progress)
    weights = toFilterWeights(abs(curvatures))

    print("start bilateral_filter...")
    filter_pcd = bilateral_filter(
        noise_pcd, sigma_d, sigma_n, filter_knn_num, weights, print_progress
    )

    saveGeometry(save_pcd_file_path, filter_pcd, overwrite, print_progress)

    filter_points = np.asarray(filter_pcd.points)

    filter_plot_figure = toPlotFigure(filter_points)

    return save_pcd_file_path, filter_plot_figure


class Server(object):
    def __init__(self, port: int) -> None:
        self.port = port

        self.input_data = None
        return

    def start(self) -> bool:
        example_folder_path = "./output/input_pcd/"
        example_file_name_list = os.listdir(example_folder_path)

        examples = [
            example_folder_path + example_file_name
            for example_file_name in example_file_name_list
        ]

        with gr.Blocks() as iface:
            gr.Markdown("PointCloud Denoise Demo")

            with gr.Row():
                with gr.Column():
                    input_pcd = gr.Model3D(label="3D Data to be denoised")

                    gr.Examples(examples=examples, inputs=input_pcd)

                    submit_button = gr.Button("Submit to server")

                output_pcd = gr.Model3D(label="BSpline Surface Sample Points")

            with gr.Row():
                with gr.Column():
                    visual_gt_plot = gr.Plot()

                    fit_button = gr.Button("Click to start fitting")

                visual_denoise_plot = gr.Plot()

            with gr.Accordion(label="Filter Params", open=False):
                sigma_d = gr.Slider(0.1, 1000.0, value=200.0, step=0.1, label="sigma_d")
                sigma_n = gr.Slider(
                    1.0, 2000.0, value=2000.0, step=0.1, label="sigma_n"
                )
                curvature_knn_num = gr.Slider(
                    1, 200, value=10, step=1, label="curvature_knn_num"
                )
                filter_knn_num = gr.Slider(
                    1, 200, value=40, step=1, label="filter_knn_num"
                )

            filter_params = [
                sigma_d,
                sigma_n,
                curvature_knn_num,
                filter_knn_num,
            ]

            submit_button.click(
                fn=renderInputData,
                inputs=[input_pcd],
                outputs=[visual_gt_plot],
            )

            fit_button.click(
                fn=toDenoisedPcd,
                inputs=[input_pcd] + filter_params,
                outputs=[output_pcd, visual_denoise_plot],
            )

        iface.launch(
            server_name="0.0.0.0",
            server_port=self.port,
            ssl_keyfile="./ssl/key.pem",
            ssl_certfile="./ssl/cert.pem",
            ssl_verify=False,
        )
        return True
