# PYTHON IMPORTS
import cv2 as cv
import numpy as np
import pathlib
import json
import glob

# COMPAS IMPORTS
from compas.geometry import Transformation, Frame, allclose
from compas_fab.robots import Configuration
from compas_fab.utilities import read_data_from_json

# LOCAL IMPORTS
from src.robot_commands import get_current_config


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


def _generate_range(folder, pose_range=False):
    """count how many config files live in folder"""

    if not pose_range:
        a = _create_file_path(folder, "")
        num_files = len(glob.glob(a.__str__() + "/*"))

        # normally +1
        pose_range = range(1, num_files + 1)
        print(pose_range)

    return pose_range


# -- SAVING DATA FUNCTIONS --#
def save_config_json(config, folder, output_file):
    """save a config for a single robot"""
    filepath = _create_file_path(folder, output_file)

    c = config.to_data()

    with open(filepath, "w") as f:
        json.dump(c, f, indent=4)


def save_config_json_multirob(rob_nums, abbs, robots, folder, filename):
    """save a config for multiple robots"""
    for abb, robot, rob_num in zip(abbs, robots, rob_nums):
        config = get_current_config(robot, abb, rob_num)

        save_config_json(
            config,
            folder=folder.format(rob_num),
            output_file=filename,
        )


def save_pnts_norm_json(pnts, folder, output_file):

    filepath = _create_file_path(folder, output_file)

    with open(filepath, "w") as f:
        json.dump(pnts, f, indent=4)


def save_frames_as_matrix_yaml(frames, folder, output_file):
    file_path = _create_file_path(folder, output_file)
    s = cv.FileStorage(file_path.__str__(), cv.FileStorage_WRITE)

    if len(frames) == 1:
        T = Transformation.from_frame(frames[0])
        PoseState = np.array(T)
        s.write("PoseState0", PoseState)
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
    s.write("PoseState0", PoseState)

    s.release()


def save_wobj_as_matrix_yaml(rob_nums, wobj_name, folder, output_file):
    """
    Create H4, transformation between WORLD0 and WOBJ
    (hard-coded wobj values from robot studio)
    """

    print(output_file)

    for rob_num in rob_nums:

        output_file2 = output_file.format(rob_num)

        if rob_num == 1:
            if wobj_name == "ACADIA":
                q1 = [0.999971, 0.000727707, -0.00129108, -0.00745658]
                f = Frame.from_quaternion(q1, point=[-680.624, 1567.49, 823.009])
            elif wobj_name == "ZEROWASTE":
                q1 = [0.999976, -0.00604606, -0.00149283, -0.0029774]
                f = Frame.from_quaternion(q1, point=[3103.33, 1481.11, 366.83])
        elif rob_num == 2:
            if wobj_name == "ACADIA":
                q1 = [0.999964, 0.00426176, -0.00158504, -0.00717332]
                f = Frame.from_quaternion(q1, point=[-681.039, 1564.71, 817.284])
            elif wobj_name == "ZEROWASTE":
                q1 = [0.99999, -0.00305874, -0.00165978, -0.0029736]
                f = Frame.from_quaternion(q1, point=[3107.65, 1478.91, 367.994])

        q2 = f.quaternion
        allclose(q1, q2, tol=1e-03)

        save_frames_as_matrix_yaml(
            frames=[f],
            folder=folder,
            output_file=output_file2,
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
            print("error in reading frame from .yaml")
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

    h = s.getNode("PoseState0").mat()

    s.release()

    T = Transformation.from_matrix(h.tolist())

    return T


def load_o3d_view_settings(folder, name):
    file_path = _create_file_path(folder, name)
    data = read_data_from_json(file_path)

    return data["trajectory"][0]


if __name__ == "__main__":
    rob_nums = [1, 2]
    save_wobj_as_matrix_yaml(
        rob_nums=rob_nums,
        wobj_name="ZEROWASTE",
        folder="transformations",
        output_file="R{}_H4_world0_rbase.yaml",
    )

    # folder = "data/stitch_shed"
    # name = "_o3d_view_settings_R{}.json".format(rob_num)
    # a = load_o3d_view_settings(folder, name)
    # print(a)
