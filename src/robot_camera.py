# COMPAS IMPORTS
import compas_rrc as rrc
from compas.geometry import Transformation

# LOCAL IMPORTS
from src.robot_commands import configs_to_move
from src.RRC_CONNECT import connect_to_robots

from src.io import (
    _create_file_path,
    _generate_range,
    load_as_transformation_yaml,
    save_frames_as_matrix_yaml,
    load_config_json,
    save_config_json_multirob,
)

from src_cam.camera.use import (
    camera_connect,
    camera_capture_settings,
)

from src_cam.camera.use import pc_downsample
from src_cam.camera.convert import convert2png


def _transform_single_pointcloud(rob_num, frame, i, folders, filenames):
    """
    transform a single point cloud for a single robot

    T2 = from calibration process (one matrix for each robot)
    T3 = the position of the TCP relative to robot base (one value per capture)
    T4 = WOBJ used in capture expressed as a transformation (one hard-coded matrix per robot)

    If you don't use T4 transformation then all pointclouds will be shown as if taken relative to the
    world-0. They have to be transformed relative to the work object.
    """

    print("\n#######FOR R{}#########".format(rob_num))

    # transformation matrix between TOOL0 and CAMERA
    T2 = load_as_transformation_yaml(folders[3], filenames[4].format(rob_num))

    # transformation matrix between ROBOT BASE (WOBJ) and TOOL0
    T3 = load_as_transformation_yaml(folders[1].format(rob_num), filenames[1].format(i))

    # transformation matrix between WORLD0 and ROBOT BASE (WOBJ)
    T4 = load_as_transformation_yaml(folders[3], filenames[5].format(rob_num))

    T = Transformation.concatenated(T4, Transformation.concatenated(T3, T2))

    pc = frame.point_cloud()
    pc.transform(T)


def _execute_aquisition(abbs, rob_nums, pose_range, folders, filenames, transform=False):
    """capture image at specified poses (need these configs saved first)"""

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

        # --save robot frame and image data for this aquisition --#
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
            except RuntimeError:
                print("--camera connect error: camera already connected")
                print("--camera connect error: or ZIVID studio is open (close it!)")

            settings = camera_capture_settings(
                camera,
                folder=folders[2],
                input_file=filenames[3].format(rob_num),
            )

            with camera.capture(settings) as frame:
                pc = frame.point_cloud()

                pc_downsample(pc, downsample_factor=4)

                if transform:  # to transform the pointcloud at capture (zerowaste not calibration)
                    _transform_single_pointcloud(rob_num, frame, i, folders, filenames)
                    frame_save_name = filenames[2].format(i) + "_trns.ply"
                else:
                    frame_save_name = filenames[2].format(i) + ".zdf"

                frame.save(_create_file_path(folders[1].format(rob_num), frame_save_name))

            _ = convert2png(
                pointcloud=pc,
                folder=folders[1].format(rob_num) + "/_imgs",
                output_file=filenames[2].format(i) + "_rgb.png",
            )

        abb.send(rrc.PrintText("IMAGE COMPLETE, press play to continue"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    print("AQUISITIONS DONE")


def robot_camera_aquisition(
    rob_nums, folders, filenames, save_config_n=False, pose_range=False, transform=False
):

    abbs, robots = connect_to_robots(rob_nums)

    if save_config_n:  # Only save configs, no camera aquisition
        save_config_json_multirob(
            rob_nums, abbs, robots, folders[0], filenames[0].format(save_config_n, width=3)
        )
    else:  # transform to world-0
        pose_range = _generate_range(folders[0].format(rob_nums[0]), pose_range)
        _execute_aquisition(abbs, rob_nums, pose_range, folders, filenames, transform)
