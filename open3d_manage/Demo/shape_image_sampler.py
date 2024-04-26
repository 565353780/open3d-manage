from open3d_manage.Module.shape_image_sampler import ShapeImageSampler


def demo():
    window_name = "Shape Image Sampler"
    width = 224
    height = 224
    left = 10
    top = 10
    visible = False
    obj_file_path = "/Users/fufu/github/OCC/mash-occ-decoder/output/mash-v2/v6.obj"
    save_folder_path = "./output/test1/"
    y_rotate_num = 8
    x_rotate_num = 5
    x_save_idxs = [1, 2, 3]
    overwrite = True

    shape_image_sampler = ShapeImageSampler(
        window_name, width, height, left, top, visible
    )
    shape_image_sampler.sampleImages(
        obj_file_path,
        save_folder_path,
        y_rotate_num,
        x_rotate_num,
        x_save_idxs,
        overwrite,
    )
    return True
