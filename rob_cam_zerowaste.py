# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robots
from src.robot_camera import robot_camera_aquisition

from src.io import (
    _generate_range,
    save_config_json_multirob,
)


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
    else:
        pose_range = _generate_range(folders[0].format(rob_nums[0]), pose_range)
        robot_camera_aquisition(abbs, rob_nums, pose_range, folders, filenames, transform)


if __name__ == "__main__":
    rob_nums = [1]

    # set "save_config_n" to FALSE to execute aquisition

    # careful 15 --> 16, and next shift to wall also
    aquisition_zerowaste(rob_nums, save_config_n=False, pose_range=range(60, 100), transform=True)
