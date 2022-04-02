# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robots
from src.robot_commands import configs_to_move, get_current_config
from src.io import save_config_json, save_frames_as_matrix_yaml, load_config_json, _generate_range

from src_cam.camera.use import (
    camera_connect,
    camera_capture_settings,
    camera_capture_and_save,
)

from src_cam.utility.io import load_pointcloud
from src_cam.camera.convert import convert2png


def _save_multi_configs(rob_nums, abbs, robots, folder, filename):
    """save a config for multiple robots"""
    for abb, robot, rob_num in zip(abbs, robots, rob_nums):
        config = get_current_config(robot, abb, rob_num)

        save_config_json(
            config,
            folder=folder.format(rob_num),
            output_file=filename,
        )


def _robot_camera_aquisition(abbs, rob_nums, pose_range, folders, filenames):
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
                print("--or ZIVID studio is open (close it!)")

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


def aquisition_calibration(rob_nums, save_config_n=False, pose_range=False):
    folders = ["configs/calibration/R{}", "data/calibration/R{}"]
    filenames = ["calibration_config_{0:0{width}}.json", "pos{:02d}.yaml", "img{:02d}"]

    abbs, robots = connect_to_robots(rob_nums)

    if save_config_n:
        _save_multi_configs(
            rob_nums, abbs, robots, folders[0], filenames[0].format(save_config_n, width=3)
        )
    else:
        pose_range = _generate_range(folders[0].format(rob_nums[0]), pose_range)
        _robot_camera_aquisition(abbs, rob_nums, pose_range, folders, filenames)


def aquisition_ECL_demo(rob_nums, save_config_n=False, pose_range=False):
    folders = ["configs/stitch_ECL_demo/R{}", "data/stitch_ECL_demo/R{}"]
    filenames = ["stitch_demo_config_{0:0{width}}.json", "pos{:02d}.yaml", "img{:02d}"]

    abbs, robots = connect_to_robots(rob_nums)

    if save_config_n:
        _save_multi_configs(
            rob_nums, abbs, robots, folders[0], filenames[0].format(save_config_n, width=3)
        )
    else:
        pose_range = _generate_range(folders[0].format(rob_nums[0]), pose_range)
        _robot_camera_aquisition(abbs, rob_nums, pose_range, folders, filenames)


def aquisition_shed(rob_nums, save_config_n=False, pose_range=False):
    folders = ["configs/stitch_shed/R{}", "data/stitch_shed/R{}"]
    filenames = ["stitch_shed_{0:0{width}}.json", "pos{:02d}.yaml", "img{:02d}"]

    abbs, robots = connect_to_robots(rob_nums)

    if save_config_n:
        _save_multi_configs(
            rob_nums, abbs, robots, folders[0], filenames[0].format(save_config_n, width=3)
        )
    else:
        pose_range = _generate_range(folders[0].format(rob_nums[0]), pose_range)
        _robot_camera_aquisition(abbs, rob_nums, pose_range, folders, filenames)


if __name__ == "__main__":
    rob_nums = [1, 2]

    # set "save_config_n" to FALSE to execute aquisition

    # aquisition_calibration(rob_nums, save_config_n=999, pose_range=range(21, 22))
    aquisition_ECL_demo(rob_nums, save_config_n=False, pose_range=range(1, 5))
    # aquisition_shed(rob_nums, save_config_n=999, pose_range=range(61, 62))
