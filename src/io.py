# PYTHON IMPORTS
import cv2 as cv
import numpy as np
import pathlib

# COMPAS IMPORTS
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_fab.robots import Configuration
from compas_fab.utilities import read_data_from_json

# LOCAL IMPORTS
from src_cam.camera.use import (
    camera_connect,
    camera_capture_settings,
    camera_capture_and_save,
)
from src_cam.camera.convert import convert2png, load_pointcloud


def _create_file_path(folder, filename, rob_num=None, i=None, **kwargs):
    """create output data path for config files.

    Returns:
        path: Output data path

    """
    path = pathlib.PurePath(
        pathlib.Path.cwd(),
        folder.format(rob_num),
        filename.format(i, width=3),
    )

    print("--created file path: ", path)
    return path


def save_frame_as_matrix_yaml(folder, name, frame, rob_num=None, i=None):
    filepath = _create_file_path(folder, name, rob_num, i)
    print(filepath)
    s = cv.FileStorage(filepath.__str__(), cv.FileStorage_WRITE)

    t = Transformation.from_frame(frame)

    PoseState = np.array(t)
    s.write("PoseState", PoseState)
    s.release()


def save_image_zdf_png(i):
    filename = "img{:02d}".format(i)

    camera = camera_connect()
    settings = camera_capture_settings(camera)
    camera_capture_and_save(
        camera,
        settings,
        "calibration_data",
        filename + ".zdf",
    )

    pc = load_pointcloud(folder="calibration_data", input_file=filename + ".zdf")

    _ = convert2png(
        pointcloud=pc,
        folder="calibration_data/_imgs",
        output_file=filename + "_rgb.png",
    )


def load_config_json(folder, name, rob_num=None, i=None):
    filename = _create_file_path(folder, name, rob_num, i)

    data = read_data_from_json(filename)
    config = Configuration.from_data(data)

    configs = []  # potentially add all configs together into one
    configs.append(config)
    return configs


def load_as_frames_yaml(folder, name):
    file_name = _create_file_path(folder, name)
    s = cv.FileStorage(file_name, cv.FILE_STORAGE_READ)

    i = 0
    H = []
    reading = True
    while reading:
        try:
            h = s.getNode("PoseState{}".format(i)).mat()

            if h[0][0]:
                H.append(h)
                i += 1

        except TypeError:
            s.release()
            reading = False

    # Make into TRANSFORMATIONS --> COMPAS frames
    F = []
    for h in H:
        h = Transformation.from_matrix(h.tolist())
        f = Frame.from_transformation(h)
        F.append(f)

    return F


def load_as_transformation_yaml(folder, name):
    file_name = _create_file_path(folder, name)
    s = cv.FileStorage(file_name, cv.FILE_STORAGE_READ)

    h = s.getNode("PoseState").mat()

    s.release()

    T = Transformation.from_matrix(h.tolist())

    return T


if __name__ == "__main__":
    _create_file_path("transformations/another", "H1_cam_obj.yaml")
    _create_file_path("calibration_data", "pos{:02d}.yaml", i=3)
    _create_file_path("calibration_data{}", "pos.yaml", rob_num=2)
