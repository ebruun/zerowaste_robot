# LOCAL IMPORTS
from src.robot_camera import robot_camera_aquisition

from src.io import _generate_range, save_pnts_norm_json

from src.process_pcd import (
    pcd_stitch_individual_rob,
    pcd_stitch_combine_rob,
    pcd_load,
    pcd_pick_points,
)


def aquisition_zerowaste(rob_nums, save_config_n=False, pose_range=False, transform=False):
    """perform camera aquisition for the full shed scanning process"""

    folders = [
        "configs/stitch_shed/R{}",
        "data/stitch_shed/R{}",
        "input_settings",
        "transformations",  # needed if transformation specified
    ]

    filenames = [
        "stitch_shed_{0:0{width}}.json",
        "pos{:02d}.yaml",
        "img{:02d}",  # if transformation, save .ply, otherwise .zdf
        "capture_settings_z{}_shed.yml",
        "R{}_H2_tool0_cam.yaml",  # needed if transformation specified (hard-coded)
        "R{}_H4_world0_rbase.yaml",  # needed if transformation specified (hard-coded)
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
            "neighbors": 20,
            "std_dev": 1.0,
            "radius": 0.08,
            "radius_pnts": 30,
        }

        # LD
        # pcd_vars = {
        #     "voxels": 0.010,
        #     "neighbors": 20,
        #     "std_dev": 1.0,
        #     "radius": 0.08,
        #     "radius_pnts": 30,
        # }

        pcd_stitch_individual_rob(rob_nums, pose_range, folders, filenames, pcd_vars, vis_on)

    if stitch_full:
        # HD
        pcd_vars = {
            "voxels": 0.001,
            "neighbors": 20,
            "std_dev": 1.0,
            "radius": 0.08,
            "radius_pnts": 30,
        }

        # LD
        # pcd_vars = {
        #     "voxels": 0.010,
        #     "neighbors": 30,
        #     "std_dev": 2.0,
        #     "radius": 0.08,
        #     "radius_pnts": 30,
        # }

        pcd_stitch_combine_rob(folders, filenames, pcd_vars, vis_on)


def select_pnts_zerowaste(member):
    folders = ["data/stitch_shed/_LD_saved", "data/stitch_shed/", "data_path_plan/"]
    filenames = [
        "_o3d_view_settings_full_door.json",
        "pcd_full_RH_cleaned.ply",
        "pcd_full_RH_cleaned.pts",
        "member_planes_{}.json".format(member),
    ]

    pcd = pcd_load(folders, filenames)
    pnts = pcd_pick_points(pcd, folders, filenames)

    for p in pnts:
        print("picked {}: dims = {} and norm = {}".format(p, pcd.points[p], pcd.normals[p]))

    save_pnts_norm_json(pnts, folders[2], filenames[3])


if __name__ == "__main__":
    rob_nums = [1]

    # set "save_config_n" to FALSE to execute aquisition

    # careful 15 --> 16, and next shift to wall also
    # careful 59 --> 60 for robot 1
    # aquisition_zerowaste(rob_nums, save_config_n=False, pose_range=range(63, 106), transform=True)

    # stitch_zerowaste(
    #     rob_nums,
    #     stitch=True,
    #     stitch_full=True,
    #     vis_on=True,
    # )

    select_pnts_zerowaste("NS4")
