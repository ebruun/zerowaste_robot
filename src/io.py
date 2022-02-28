# PYTHON IMPORTS
import cv2 as cv
import numpy as np
import pathlib
import json

# COMPAS IMPORTS
from compas.geometry import Frame
from compas.geometry import Transformation
from compas_fab.robots import Configuration
from compas_fab.utilities import read_data_from_json


def _create_file_path(folder, filename, rob_num=None, i=None):
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


# -- SAVING DATA FUNCTIONS --#
def save_config_json(config, folder, output_file):
    filepath = _create_file_path(folder, output_file)

    c = config.to_data()

    with open(filepath, "w") as f:
        json.dump(c, f, indent=4)


def save_frames_as_matrix_yaml(frames, folder, output_file):
    file_path = _create_file_path(folder, output_file)
    s = cv.FileStorage(file_path.__str__(), cv.FileStorage_WRITE)

    if len(frames) == 1:
        T = Transformation.from_frame(frames[0])
        PoseState = np.array(T)
        s.write("PoseState", PoseState)
    else:
        for i, f in enumerate(frames):
            T = Transformation.from_frame(f)
            PoseState = np.array(T)
            s.write("PoseState{}".format(i), PoseState)

    s.release()


# -- LOADING DATA FUNCTIONS --#
def load_config_json(folder, name):
    filename = _create_file_path(folder, name)

    data = read_data_from_json(filename)
    config = Configuration.from_data(data)

    configs = []  # potentially add all configs together into one
    configs.append(config)
    return configs


def load_as_frames_yaml(folder, name):
    file_name = _create_file_path(folder, name)
    s = cv.FileStorage(file_name.__str__(), cv.FILE_STORAGE_READ)

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
    s = cv.FileStorage(file_name.__str__(), cv.FILE_STORAGE_READ)

    h = s.getNode("PoseState").mat()

    s.release()

    T = Transformation.from_matrix(h.tolist())

    return T


if __name__ == "__main__":
    _create_file_path("transformations/another", "H1_cam_obj.yaml")
    _create_file_path("calibration_data", "pos{:02d}.yaml", i=3)
    _create_file_path("calibration_data{}", "pos.yaml", rob_num=2)
