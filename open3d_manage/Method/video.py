import os
import re
import cv2
import numpy as np
from tqdm import tqdm

from open3d_manage.Method.path import createFileFolder

def createVideoFromImages(image_folder_path: str,
                          save_video_file_path: str,
                          bg_color: list = [255, 255, 255],
                          fps: int = 30,
                          overwrite: bool = False) -> bool:
    if not overwrite:
        if os.path.exists(save_video_file_path):
            return True

    createFileFolder(save_video_file_path)

    image_filename_list = os.listdir(image_folder_path)
    image_filename_list.sort(key=lambda x: int(re.findall(r'\d+', os.path.splitext(x)[0])[0]))

    image_filepath_list = []

    for image_filename in image_filename_list:
        if image_filename[-4:] != ".png":
            continue

        image_file_path = image_folder_path + image_filename

        image_filepath_list.append(image_file_path)

    first_image = cv2.imread(image_filepath_list[0], cv2.IMREAD_UNCHANGED)
    height, width = first_image.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(save_video_file_path, fourcc, fps, (width, height))

    print('[INFO][video::createVideoFromImages]')
    print('\t start convert images to video...')
    for image_file_path in tqdm(image_filepath_list):
        image = cv2.imread(image_file_path, cv2.IMREAD_UNCHANGED)

        if image.shape[2] == 3:
            video_writer.write(image)
            continue

        bgr = image[:, :, :3]
        alpha = image[:, :, 3]

        background = np.asarray(bg_color, dtype=np.uint8)

        alpha_mask = alpha / 255.0

        bgr = bgr * alpha_mask[:, :, np.newaxis] + background * (1 - alpha_mask[:, :, np.newaxis])
        bgr = bgr.astype(np.uint8)

        video_writer.write(bgr)

    video_writer.release()
    return True
