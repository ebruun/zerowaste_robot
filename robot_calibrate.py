# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.robot_commands import configs_to_move
from src.io import (
    save_config_json,
    save_frame_as_matrix_yaml,
    save_image_zdf_png,
    load_config_json,
)


def get_current_config(robot, abb, rob_num):

    config = robot.zero_configuration()
    print(config)

    if rob_num == 1:
        current_joints, current_track = abb.send_and_wait(rrc.GetJoints(), timeout=3)

        config["r1_cart_joint"] = current_track[0]
        config["r1_joint_1"] = current_joints[0]
        config["r1_joint_2"] = current_joints[1]
        config["r1_joint_3"] = current_joints[2]
        config["r1_joint_4"] = current_joints[3]
        config["r1_joint_5"] = current_joints[4]
        config["r1_joint_6"] = current_joints[5]

    elif rob_num == 2:
        current_joints, current_track = abb.send_and_wait(rrc.GetJoints(), timeout=3)

        config["r2_cart_joint"] = current_track[0]
        config["r2_joint_1"] = current_joints[0]
        config["r2_joint_2"] = current_joints[1]
        config["r2_joint_3"] = current_joints[2]
        config["r2_joint_4"] = current_joints[3]
        config["r2_joint_5"] = current_joints[4]
        config["r2_joint_6"] = current_joints[5]

    return config


def calibration(abbs, rob_nums, pose_range):
    print("START CALIBRATION")

    for i in pose_range:
        print("\nCALIBRATION POSE #{}\n".format(i))

        configs = []

        # --read in pre-saved calibration config --#
        for abb, rob_num in zip(abbs, rob_nums):

            configs.append(
                load_config_json(
                    "configs/calibration/R{}",
                    "calibration_config_{0:0{width}}.json",
                    rob_num,
                    i,
                )
            )

        # --move to pre-saved calibration config --#
        for abb, rob_num, config in zip(abbs, rob_nums, configs):
            configs_to_move(abb, rob_num, config)

        abb.send(rrc.PrintText("MOVE CONFIG_{0:03} DONE, play for image".format(i)))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        # --save frame and image data to be used for calibration --#
        abb.send(rrc.PrintText("Saving frame and taking image..."))
        for abb, rob_num in zip(abbs, rob_nums):
            print("\nTAKING IMAGE WITH R{}".format(rob_num))
            f_at_config = abb.send_and_wait(rrc.GetFrame(), timeout=3)

            save_frame_as_matrix_yaml(
                "calibration_data/R{}", "pos{:02d}.yaml", f_at_config, rob_num, i
            )
            save_image_zdf_png(i, rob_num)

        abb.send(rrc.PrintText("IMAGE COMPLETE, press play to continue"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    print("\nCALIBRATION DONE")


if __name__ == "__main__":
    # robot, abb = connect_to_robot(rob_num=1)
    # config = get_current_config(robot, abb, rob_num=1)
    # save_config_json("configs/calibration/R{}","calibration_config_{0:0{width}}.json", config, rob_num=1, i=300)

    # # CALIBRATE BOTH AT SAME TIME
    # robot1, abb1 = connect_to_robot(rob_num=1)
    # robot2, abb2 = connect_to_robot(rob_num=2)
    # calibration([abb1, abb2], rob_nums=[1, 2], pose_range=range(1, 30))

    # CALIBRATE SINGLE ROBOT
    robot, abb = connect_to_robot(rob_num=1)
    calibration([abb], rob_nums=[1], pose_range=range(20, 21))
