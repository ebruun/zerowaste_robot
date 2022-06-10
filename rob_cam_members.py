# PYTHON IMPORTS
import json

# LOCAL IMPORTS
from src.robot_camera import robot_camera_aquisition

from src.io import _generate_range

from src.process_pcd import (
    pcd_stitch_individual_rob,
    pcd_stitch_combine_rob,
)


def aquisition_members(rob_nums, save_config_n=False, pose_range=False, transform=False):
    folders = ["configs/stitch_members/R{}", "data/stitch_members/R{}", "transformations"]
    filenames = [
        "stitch_members_{0:0{width}}.json",
        "pos{:02d}.yaml",
        "img{:02d}",
        "R{}_H2_tool0_cam.yaml",
        "R{}_H4_world0_rbase.yaml",
        "img{:02d}_trns.ply",
    ]

    robot_camera_aquisition(rob_nums, folders, filenames, save_config_n, pose_range, transform)


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
        "_o3d_view_settings_full.json",
        "pcd_full.pts",
    ]

    pose_range = _generate_range(folders[1].format(rob_nums[0]), pose_range)

    if stitch:
        # HD
        pcd_vars = {
            "voxels": 0.001,
            "neighbors": 30,
            "std_dev": 1.0,
            "radius": 0.08,
            "radius_pnts": 30,
        }

        # LD
        pcd_vars = {
            "voxels": 0.010,
            "neighbors": 20,
            "std_dev": 1.0,
            "radius": 0.08,
            "radius_pnts": 30,
        }

        pcd_stitch_individual_rob(rob_nums, pose_range, folders, filenames, pcd_vars, vis_on)

    if stitch_full:
        # HD
        pcd_vars = {
            "voxels": 0.001,
            "neighbors": 30,
            "std_dev": 2.0,
            "radius": 0.08,
            "radius_pnts": 30,
        }

        # LD
        pcd_vars = {
            "voxels": 0.010,
            "neighbors": 30,
            "std_dev": 2.0,
            "radius": 0.08,
            "radius_pnts": 30,
        }

        pcd_stitch_combine_rob(folders, filenames, pcd_vars, vis_on)


if __name__ == "__main__":
    rob_nums = [1]

    # set "save_config_n" to FALSE to execute aquisition

    aquisition_members(rob_nums, save_config_n=1, pose_range=range(62, 63), transform=True)

    # stitch_zerowaste(
    #     rob_nums,
    #     stitch=False,
    #     stitch_full=True,
    #     vis_on=True,
    # )
