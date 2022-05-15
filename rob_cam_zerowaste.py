# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robots
from src.robot_camera import robot_camera_aquisition

from src.io import (
    _generate_range,
    save_config_json_multirob,
)

from src.robot_stitch import stitch_pcd_individual_rob, stitch_pcd_combine_rob


def aquisition_zerowaste(rob_nums, save_config_n=False, pose_range=False, transform=False):
    folders = ["configs/stitch_shed/R{}", "data/stitch_shed/R{}", "transformations"]
    filenames = [
        "stitch_shed_{0:0{width}}.json",
        "pos{:02d}.yaml",
        "img{:02d}",
        "R{}_H2_tool0_cam.yaml",
        "R{}_H4_world0_rbase.yaml",
        "img{:02d}_trns.ply",
    ]

    abbs, robots = connect_to_robots(rob_nums)

    if save_config_n:  # Only save configs, no camera aquisition
        save_config_json_multirob(
            rob_nums, abbs, robots, folders[0], filenames[0].format(save_config_n, width=3)
        )
    else:  # transform to world-0
        pose_range = _generate_range(folders[0].format(rob_nums[0]), pose_range)
        robot_camera_aquisition(abbs, rob_nums, pose_range, folders, filenames, transform)


def stitch_zerowaste(rob_nums, stitch=False, stitch_full=False, pose_range=False, vis_on=False):
    folders = [
        "transformations",
        "configs/stitch_shed/R{}",
        "data/stitch_shed/R{}",
        "data/stitch_shed",
    ]
    filenames = [
        "img{:02d}.zdf",
        "img{:02d}_trns.ply",
        "R{}_H2_tool0_cam.yaml",
        "pos{:02d}.yaml",
        "R{}_H4_world0_rbase.yaml",
        "_R{}_pcd_stitched.pts",
        "_o3d_view_settings_R{}.json",
        "_o3d_view_settings_full2.json",
        "pcd_full.pts",
    ]

    pose_range = _generate_range(folders[1].format(rob_nums[0]), pose_range)

    if stitch:
        stitch_pcd_individual_rob(rob_nums, pose_range, folders, filenames, vis_on)

    if stitch_full:
        stitch_pcd_combine_rob(folders, filenames, vis_on)


if __name__ == "__main__":
    rob_nums = [1]

    # set "save_config_n" to FALSE to execute aquisition

    # careful 15 --> 16, and next shift to wall also
    aquisition_zerowaste(rob_nums, save_config_n=False, pose_range=range(60, 100), transform=True)

    stitch_zerowaste(
        rob_nums,
        stitch=True,
        stitch_full=False,
        pose_range=range(1, 100),
        vis_on=True,
    )
