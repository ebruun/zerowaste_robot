# PYTHON IMPORTS
import zivid

# COMPAS IMPORTS
from src.robot_camera import robot_camera_aquisition

from src.io import (
    load_as_transformation_yaml,
    save_transformation_as_matrix_yaml,
)

from src_cam.utility.io import load_pointcloud


def aquisition_calibration(rob_nums, save_config_n=False, pose_range=False):
    """perform camera aquisition for the checkerboard calibration process"""

    folders = [
        "configs/calibration/R{}",
        "data/calibration/R{}",
        "input_settings",
        # no tranformation folder needed
    ]

    filenames = [
        "calibration_config_{0:0{width}}.json",
        "pos{:02d}.yaml",
        "img{:02d}",
        "capture_settings_calibration_z{}.yml",
        # no transformation filename needed
    ]

    robot_camera_aquisition(rob_nums, folders, filenames, save_config_n, pose_range)


def perform_calibration(rob_nums, pose_range):
    folders = ["data/calibration/R{}", "transformations"]
    filenames = ["pos{:02d}.yaml", "img{:02d}.zdf", "R{}_H2_tool0_cam.yaml"]

    for rob_num in rob_nums:

        hand_eye_input = []

        for i in pose_range:
            pc, _ = load_pointcloud(
                folder=folders[0].format(rob_num),
                input_file=filenames[1].format(i),
            )

            t = load_as_transformation_yaml(
                folder=folders[0].format(rob_num), name=filenames[0].format(i)
            )

            detection_result = zivid.calibration.detect_feature_points(pc)
            pose = zivid.calibration.Pose(t)

            if detection_result:
                print("pose {0:0{width}} success".format(i, width=3))
                hand_eye_input.append(zivid.calibration.HandEyeInput(pose, detection_result))
            else:
                print("pose {0:0{width}} failure".format(i, width=3))

        hand_eye_output = zivid.calibration.calibrate_eye_in_hand(hand_eye_input)

        if hand_eye_output.valid():
            print("Hand-Eye calibration OK")
            print(f"Result:\n{hand_eye_output.transform()}")
        else:
            print("Hand-Eye calibration FAILED")

        save_transformation_as_matrix_yaml(
            transformation=hand_eye_output.transform(),
            folder=folders[1],
            output_file=filenames[2].format(rob_num),
        )


if __name__ == "__main__":

    rob_nums = [1, 2]

    # aquisition_calibration(rob_nums, save_config_n=False, pose_range=range(1, 31))
    perform_calibration(rob_nums, pose_range=range(1, 31))
