# PYTHON IMPORTS
import cv2 as cv
import numpy as np
import pathlib
import json

# COMPAS IMPORTS
from compas.geometry import Transformation, Frame, allclose
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


def save_transformation_as_matrix_yaml(transformation, folder, output_file):
    file_path = _create_file_path(folder, output_file)
    s = cv.FileStorage(file_path.__str__(), cv.FileStorage_WRITE)

    PoseState = np.array(transformation)
    s.write("PoseState", PoseState)

    s.release()


# Create wobj to world 0 transform (hard-coded values from robot studio)
def save_wobj_as_matrix_yaml(rob_num, folder, output_file):

    if rob_num == 1:
        q1 = [0.999971, 0.000727707, -0.00129108, -0.00745658]
        f = Frame.from_quaternion(q1, point=[-680.624, 1567.49, 823.009])
        q2 = f.quaternion
        allclose(q1, q2, tol=1e-03)
    elif rob_num == 2:
        q1 = [0.999964, 0.00426176, -0.00158504, -0.00717332]
        f = Frame.from_quaternion(q1, point=[-681.039, 1564.71, 817.284])
        q2 = f.quaternion
        allclose(q1, q2, tol=1e-03)

    save_frames_as_matrix_yaml(
        frames=[f],
        folder=folder,
        output_file=output_file,
    )


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


def load_o3d_view_settings(folder, name):
    file_path = _create_file_path(folder, name)
    data = read_data_from_json(file_path)

    return data["trajectory"][0]


if __name__ == "__main__":
    # _create_file_path("transformations/another", "H1_cam_obj.yaml")
    # _create_file_path("calibration_data", "pos{:02d}.yaml", i=3)
    # _create_file_path("calibration_data{}", "pos.yaml", rob_num=2)

    rob_num = 2
    # save_wobj_as_matrix_yaml(rob_num,"transformations","R{}_H4_world_wobj.yaml".format(rob_num))

    # folder = "data/stitch_shed"
    # name = "_o3d_view_settings_R{}.json".format(rob_num)
    # a = load_o3d_view_settings(folder, name)
    # print(a)
