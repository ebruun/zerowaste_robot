# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.robot_commands import configs_to_move
from src.io import (
    save_config_json,
    save_frames_as_matrix_yaml,
    load_config_json,
)

from src_cam.camera.use import (
    camera_connect,
    camera_capture_settings,
    camera_capture_and_save,
)

from src_cam.utility.io import load_pointcloud
from src_cam.camera.convert import convert2png


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


def calibration(abbs, rob_nums, pose_range, folders, filenames):
    print("START AQUISITION PROCESS")

    for i in pose_range:
        print("\nAQUISITION POSE #{}\n".format(i))

        configs = []

        # --read in pre-saved calibration config --#
        for abb, rob_num in zip(abbs, rob_nums):

            configs.append(
                load_config_json(
                    folders[0].format(rob_num),
                    filenames[0].format(i, width=3),
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
            f_at_config = [abb.send_and_wait(rrc.GetFrame(), timeout=3)]

            save_frames_as_matrix_yaml(
                frames=f_at_config,
                folder=folders[1].format(rob_num),
                output_file=filenames[1].format(i),
            )

            try:
                camera = camera_connect(rob_num)
                settings = camera_capture_settings(camera)
            except RuntimeError:
                print("--camera already connected")

            camera_capture_and_save(
                camera,
                settings,
                folder=folders[1].format(rob_num),
                output_file=filenames[2].format(i) + ".zdf",
            )

            pc, _ = load_pointcloud(
                folder=folders[1].format(rob_num),
                input_file=filenames[2].format(i) + ".zdf",
            )

            _ = convert2png(
                pointcloud=pc,
                folder=folders[1].format(rob_num) + "/_imgs",
                output_file=filenames[2].format(i) + "_rgb.png",
            )

        abb.send(rrc.PrintText("IMAGE COMPLETE, press play to continue"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    print("AQUISITIONS DONE")


if __name__ == "__main__":
    # rob_num=1
    # robot, abb = connect_to_robot(rob_num)
    # config = get_current_config(robot, abb, rob_num)
    # save_config_json(
    #     config,
    #     folder="configs/stitch/R{}".format(rob_num),
    #     output_file="stitch_config_{0:0{width}}.json".format(5,width=3)
    # )

    # # CALIBRATE BOTH AT SAME TIME
    # robot1, abb1 = connect_to_robot(rob_num=1)
    # robot2, abb2 = connect_to_robot(rob_num=2)
    # calibration([abb1, abb2], rob_nums=[1, 2], pose_range=range(1, 30))

    # # CALIBRATE SINGLE ROBOT
    # robot, abb = connect_to_robot(rob_num=1)
    # calibration([abb], rob_nums=[1], pose_range=range(1, 2))

    # STITCH DATA SINGLE ROBOT
    robot, abb = connect_to_robot(rob_num=1)
    calibration(
        [abb],
        rob_nums=[1],
        pose_range=range(4, 6),
        folders=["configs/stitch/R{}", "data_stitch/R{}"],
        filenames=["stitch_config_{0:0{width}}.json", "pos{:02d}.yaml", "img{:02d}"],
    )
