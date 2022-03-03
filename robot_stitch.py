import open3d as o3d
import numpy as np

# COMPAS IMPORTS
import compas_rrc as rrc
from compas.geometry import Transformation

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.robot_commands import configs_to_move
from src.io import (
    load_as_transformation_yaml,
    _create_file_path,
)

from src_cam.utility.io import load_pointcloud


def _transform_pointclouds(rob_nums, pose_range, folders, filenames):

    print("START POINTCLOUD TRANSFORM\n")

    for rob_num in rob_nums:
        print("\n#######FOR R{}#########".format(rob_num))
        for i in pose_range:
            print("\nLOAD AND TRANSFORM POSE {}".format(i))

            pc, frame = load_pointcloud(
                folder=folders[0].format(rob_num),
                input_file=filenames[0].format(i, width=3),
            )

            # transformation matrix between robot and camera
            T2 = load_as_transformation_yaml(folders[2], filenames[1].format(rob_num))

            # transformation matrix between robot and wobj (tool0 frame)
            T3 = load_as_transformation_yaml(folders[0].format(rob_num), filenames[3].format(i))

            # transformation matrix between wobj and world-0
            T4 = load_as_transformation_yaml(folders[2], filenames[2].format(rob_num))

            T = Transformation.concatenated(T4, Transformation.concatenated(T3, T2))

            pc.transform(T)
            frame.save(_create_file_path(folders[0].format(rob_num), filenames[4].format(i)))

    print("\nPOINTCLOUD TRANSFORMS DONE")


def stitch_pcd(rob_nums, pc_range, voxel_size, folders, filenames):

    print("START POINTCLOUD STITCH\n")

    for rob_num in rob_nums:
        point_data = []
        color_data = []
        for i in pc_range:
            pcd = o3d.io.read_point_cloud(
                _create_file_path(
                    folder=folders[0].format(rob_num), filename=filenames[4].format(i)
                ).__str__()
            )

            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=0.5)

            p = np.asarray(pcd.points)
            p_color = np.asarray(pcd.colors)

            point_data.append(p)
            color_data.append(p_color)

        points_combined = np.concatenate(point_data, axis=0) / 1000  # mm -> m
        colors_combined = np.concatenate(color_data, axis=0)

        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
        pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

        o3d.visualization.draw_geometries(
            [pcd_combined],
            zoom=0.399,
            front=[-0.702, 0.493, 0.512],
            lookat=[-0.141, 1.503, 0.5210],
            up=[0.4154, -0.3007, 0.8584],
        )

        o3d.io.write_point_cloud(
            _create_file_path(folder=folders[1], filename=filenames[6].format(rob_num)).__str__(),
            pcd_combined,
        )

    print("\nPOINTCLOUD STITCH DONE")


if __name__ == "__main__":

    folders = ["data/stitch_shed/R{}", "data/stitch_shed", "transformations"]
    filenames = [
        "img{:02d}.zdf",
        "R{}_H2_robot_cam.yaml",
        "R{}_H4_world_wobj.yaml",
        "pos{:02d}.yaml",
        "img{:02d}_trns.ply",
        "img_notrans{:02d}.ply",
        "R{}_pcd_stitched.pts",
    ]

    rob_nums = [1, 2]
    # _transform_pointclouds(
    #     rob_nums=rob_nums,
    #     pose_range=range(1, 7),
    #     folders=folders,
    #     filenames=filenames
    # )

    stitch_pcd(
        rob_nums=rob_nums, pc_range=range(1, 7), voxel_size=1, folders=folders, filenames=filenames
    )
