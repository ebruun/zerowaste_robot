# PYTHON IMPORTS
import zivid

# COMPAS IMPORTS
from src.io import load_as_transformation_yaml, save_transformation_as_matrix_yaml

from src_cam.utility.io import load_pointcloud


def perform_calibration(rob_nums, pose_range):
    folders = ["data/calibration/R{}", "transformations"]
    filenames = ["pos{:02d}.yaml", "img{:02d}.zdf", "R{}_H2_robot_cam.yaml"]

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
    perform_calibration(rob_nums, pose_range=range(1, 31))
