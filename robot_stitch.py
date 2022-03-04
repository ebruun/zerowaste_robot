import open3d as o3d
import numpy as np

# COMPAS IMPORTS
import compas_rrc as rrc
from compas.geometry import Transformation

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.robot_commands import configs_to_move
from src.io import load_as_transformation_yaml, _create_file_path, load_o3d_view_settings

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


def _visualize_pcd(pcd, folder, filename):
    vis_settings = load_o3d_view_settings(folder, filename)
    o3d.visualization.draw_geometries(
        [pcd],
        left=10,
        top=50,
        width=1600,
        height=900,
        zoom=vis_settings["zoom"],
        front=vis_settings["front"],
        lookat=vis_settings["lookat"],
        up=vis_settings["up"],
    )


def stitch_separate_robots(rob_nums, pc_range, vis_on=False):
    folders = ["data/stitch_shed/R{}"]
    filenames = [
        "img{:02d}_trns.ply",
        "_R{}_pcd_stitched.pts",
        "_o3d_view_settings_R{}.json",
    ]

    pcd_vars = {
        "voxels": 8,
        "neighbors": 20,
        "std_dev": 0.1,
        "scale": 1000,
    }

    print("START POINTCLOUD STITCH\n")
    for rob_num in rob_nums:
        point_data = []
        color_data = []

        for i in pc_range:
            pcd = o3d.io.read_point_cloud(
                _create_file_path(
                    folder=folders[0].format(rob_num), filename=filenames[0].format(i)
                ).__str__()
            )

            pcd = pcd.voxel_down_sample(voxel_size=pcd_vars["voxels"])
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=pcd_vars["neighbors"], std_ratio=pcd_vars["std_dev"]
            )

            point_data.append(np.asarray(pcd.points))
            color_data.append(np.asarray(pcd.colors))

        points_combined = np.concatenate(point_data, axis=0) / pcd_vars["scale"]  # mm -> m
        colors_combined = np.concatenate(color_data, axis=0)

        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
        pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

        if vis_on:
            _visualize_pcd(
                pcd_combined,
                folders[0].format(rob_num),
                filenames[2].format(rob_num),
            )

        o3d.io.write_point_cloud(
            _create_file_path(
                folder=folders[0].format(rob_num), filename=filenames[1].format(rob_num)
            ).__str__(),
            pcd_combined,
        )


def stitch_full(vis_on=False):
    folders = ["data/stitch_shed/R{}", "data/stitch_shed"]
    filenames = [
        "_R{}_pcd_stitched.pts",
        "pcd_full.pts",
        "_o3d_view_settings_full.json",
    ]

    pcd_vars = {
        "voxels": 0.004,
        "neighbors": 5,
        "std_dev": 1.0,
        "scale": 1,
    }

    print("START FULL POINTCLOUD STITCH\n")

    rob_nums = [1, 2]
    point_data = []
    color_data = []
    for rob_num in rob_nums:
        pcd = o3d.io.read_point_cloud(
            _create_file_path(
                folder=folders[0].format(rob_num), filename=filenames[0].format(rob_num)
            ).__str__()
        )

        point_data.append(np.asarray(pcd.points))
        color_data.append(np.asarray(pcd.colors))

    points_combined = np.concatenate(point_data, axis=0) / pcd_vars["scale"]  # mm -> m
    colors_combined = np.concatenate(color_data, axis=0)

    pcd_combined = o3d.geometry.PointCloud()
    pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
    pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

    pcd_combined = pcd_combined.voxel_down_sample(voxel_size=pcd_vars["voxels"])
    # pcd_combined, _ = pcd_combined.remove_statistical_outlier(nb_neighbors=pcd_vars['neighbors'], std_ratio=pcd_vars['std_dev'])

    if vis_on:
        _visualize_pcd(
            pcd_combined,
            folders[1].format(rob_num),
            filenames[2],
        )

    o3d.io.write_point_cloud(
        _create_file_path(
            folder=folders[1].format(rob_num), filename=filenames[1].format(rob_num)
        ).__str__(),
        pcd_combined,
    )


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
        "_o3d_view_settings_R{}.json",
        "full_pcd_stitched.pts",
    ]

    rob_nums = [1, 2]
    # _transform_pointclouds(
    #     rob_nums=rob_nums,
    #     pose_range=range(60, 100),
    #     folders=folders,
    #     filenames=filenames
    # )

    # stitch_separate_robots(
    #     rob_nums=rob_nums,
    #     pc_range=range(1, 100),
    #     vis_on=False,
    # )

    stitch_full(vis_on=True)
