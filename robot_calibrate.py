# Before running this example, make sure to run
# "docker compose up" on the docker/moveit folder

import cv2 as cv
import numpy as np
import os

# COMPAS IMPORTS
import compas_rrc as rrc
from compas_fab.robots import Configuration
from compas_fab.utilities import read_data_from_json
from compas.geometry import Transformation

# LOCAL IMPORTS
from src.camera.use import capture_image
from src.camera.convert import convert2png, load_pointcloud


# Load robot without geometry
def _robot_connect(rob_num):

    # with RosClient('localhost') as ros:

    ros = rrc.RosClient()
    ros.run()

    robot = ros.load_robot()

    # --set Robot to connect to-- #
    if rob_num == 1:
        r = '/rob1'
    elif rob_num == 2:
        r = '/rob2'

    abb = rrc.AbbClient(ros, r)
    abb.send(rrc.PrintText("CONNECTING TO ROBOT_{} press play to continue".format(rob_num)))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    # --Set work object-- #
    abb.send(rrc.SetWorkObject('rob2_acadia_EBIT_01'))
    # abb.send(rrc.SetWorkObject('ECL_platform'))
    # abb.send(rrc.SetWorkObject('wobj0'))

    # --Execute command-- #
    abb.send(rrc.PrintText('Starting...'))

    return robot, abb


def _build_config_path(rob_num, i):

    path = os.path.join(os.getcwd(), 'calibration_configs\\calibration_configs_R{}\\'.format(rob_num))  # folder
    path = os.path.join(path, 'calibration_config_{0:0{width}}.json'.format(i, width=3))  # file
    return path


def _read_saved_config(p):
    data = read_data_from_json(p)
    config = Configuration.from_data(data)

    configs = []  # potentially add all configs together into one
    configs.append(config)
    return configs


def move_to_config(rob_num, abb, configs):
    abb.send(rrc.PrintText("Moving..."))
    speed = 100

    for config in configs:
        if rob_num == 1:
            axis = rrc.ExternalAxes(config['r1_cart_joint'])
            joints = rrc.RobotJoints(
                config['r1_joint_1'],
                config['r1_joint_2'],
                config['r1_joint_3'],
                config['r1_joint_4'],
                config['r1_joint_5'],
                config['r1_joint_6'],
            )
        elif rob_num == 2:
            axis = rrc.ExternalAxes(config['r2_cart_joint'])
            joints = rrc.RobotJoints(
                config['r2_joint_1'],
                config['r2_joint_2'],
                config['r2_joint_3'],
                config['r2_joint_4'],
                config['r2_joint_5'],
                config['r2_joint_6'],
            )

        abb.send(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE))


def generate_frame_data(i, frame):
    filename = "calibration_data/pos{:02d}.yaml".format(i)
    s = cv.FileStorage(filename, cv.FileStorage_WRITE)

    t = Transformation.from_frame(frame)

    PoseState = np.array(t)
    s.write('PoseState', PoseState)
    s.release()


def generate_image_data(i):
    filename = "img{:02d}".format(i)

    capture_image(
        folder="calibration_data",
        output_file=filename + ".zdf",
        setting_file="detection_settings.yml",
        )

    pc = load_pointcloud(
        folder="calibration_data",
        input_file=filename + ".zdf"
    )

    _ = convert2png(
        pointcloud=pc,
        folder="calibration_img",
        output_file=filename + "_rgb.png",

        )


def calibration(rob_num, num_poses):
    print("START CALIBRATION")

    robot, abb = _robot_connect(rob_num)

    for i in range(num_poses):
        print('\n--calibration pose #{}\n'.format(i+1))

        path = _build_config_path(rob_num, i+1)
        config = _read_saved_config(path)
        move_to_config(rob_num, abb, config)

        abb.send(rrc.PrintText("MOVE COMPLETE, press play to take image"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        f = abb.send_and_wait(rrc.GetFrame(), timeout=3)
        print("--frame: ", f)

        generate_frame_data(i+1, f)
        generate_image_data(i+1)

        abb.send(rrc.PrintText("IMAGE COMPLETE, press play to continue"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    print("\nCALIBRATION DONE")


if __name__ == "__main__":
    # calibration variables
    rob_num = 2
    num_calibration_poses = 2

    calibration(rob_num, num_calibration_poses)
