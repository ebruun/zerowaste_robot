import open3d as o3d
import numpy as np

# COMPAS IMPORTS
from compas.geometry import Transformation

# LOCAL IMPORTS
from src.io import (
    load_as_transformation_yaml,
    _create_file_path,
    _generate_range,
    load_o3d_view_settings,
)

from src_cam.utility.io import load_pointcloud


def _transform_single_pointcloud(rob_num, frame, i, folders, filenames):

    print("\n#######FOR R{}#########".format(rob_num))

    # transformation matrix between TOOL0 and CAMERA
    T2 = load_as_transformation_yaml(folders[2], filenames[3].format(rob_num))

    # transformation matrix between ROBOT BASE (WOBJ) and TOOL0
    T3 = load_as_transformation_yaml(folders[1].format(rob_num), filenames[1].format(i))

    # transformation matrix between WORLD0 and ROBOT BASE (WOBJ)
    T4 = load_as_transformation_yaml(folders[2], filenames[4].format(rob_num))

    T = Transformation.concatenated(T4, Transformation.concatenated(T3, T2))

    pc = frame.point_cloud()
    pc.transform(T)


def _transform_pointclouds(rob_nums, pose_range, folders, filenames):

    print("START POINTCLOUD TRANSFORM\n")

    for rob_num in rob_nums:
        print("\n#######FOR R{}#########".format(rob_num))
        for i in pose_range:
            print("\nLOAD AND TRANSFORM POSE {}".format(i))

            pc, frame = load_pointcloud(
                folder=folders[2].format(rob_num),
                input_file=filenames[0].format(i, width=3),
            )

            # transformation matrix between TOOL0 and CAMERA
            T2 = load_as_transformation_yaml(folders[0], filenames[2].format(rob_num))

            # transformation matrix between ROBOT BASE (WOBJ) and TOOL0
            T3 = load_as_transformation_yaml(folders[2].format(rob_num), filenames[3].format(i))

            # transformation matrix between WORLD0 and ROBOT BASE (WOBJ)
            T4 = load_as_transformation_yaml(folders[0], filenames[4].format(rob_num))

            T = Transformation.concatenated(T4, Transformation.concatenated(T3, T2))

            pc.transform(T)
            frame.save(_create_file_path(folders[2].format(rob_num), filenames[1].format(i)))

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


def _stitch_pcd(rob_nums, pc_range, folders, filenames, vis_on=False):
    pcd_vars = {
        "voxels": 0.004,
        "neighbors": 30,
        "std_dev": 1.0,
        "radius": 0.08,
        "radius_pnts": 30,
    }

    print("START POINTCLOUD STITCH\n")
    for rob_num in rob_nums:
        point_data = []
        color_data = []

        for i in pc_range:
            pcd = o3d.io.read_point_cloud(
                _create_file_path(
                    folder=folders[2].format(rob_num), filename=filenames[1].format(i)
                ).__str__()
            )

            point_data.append(np.asarray(pcd.points))
            color_data.append(np.asarray(pcd.colors))

        points_combined = np.concatenate(point_data, axis=0) / 1000  # mm -> m
        colors_combined = np.concatenate(color_data, axis=0)

        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
        pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

        pcd_combined = pcd_combined.voxel_down_sample(voxel_size=pcd_vars["voxels"])
        pcd_combined, _ = pcd_combined.remove_statistical_outlier(
            nb_neighbors=pcd_vars["neighbors"], std_ratio=pcd_vars["std_dev"]
        )
        pcd_combined, _ = pcd_combined.remove_radius_outlier(
            nb_points=pcd_vars["radius_pnts"], radius=pcd_vars["radius"]
        )

        if vis_on:
            _visualize_pcd(
                pcd_combined,
                folders[2].format(rob_num),
                filenames[6].format(rob_num),
            )

        o3d.io.write_point_cloud(
            _create_file_path(
                folder=folders[2].format(rob_num), filename=filenames[5].format(rob_num)
            ).__str__(),
            pcd_combined,
        )


def _stitch_full(folders, filenames, vis_on=False):
    pcd_vars = {
        "voxels": 0.004,
        "neighbors": 30,
        "std_dev": 1.0,
        "radius": 0.08,
        "radius_pnts": 30,
    }

    print("START FULL POINTCLOUD STITCH\n")

    rob_nums = [1, 2]
    point_data = []
    color_data = []
    for rob_num in rob_nums:
        pcd = o3d.io.read_point_cloud(
            _create_file_path(
                folder=folders[2].format(rob_num), filename=filenames[5].format(rob_num)
            ).__str__()
        )

        point_data.append(np.asarray(pcd.points))
        color_data.append(np.asarray(pcd.colors))

    points_combined = np.concatenate(point_data, axis=0)
    colors_combined = np.concatenate(color_data, axis=0)

    pcd_combined = o3d.geometry.PointCloud()
    pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
    pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

    pcd_combined = pcd_combined.voxel_down_sample(voxel_size=pcd_vars["voxels"])
    pcd_combined, _ = pcd_combined.remove_statistical_outlier(
        nb_neighbors=pcd_vars["neighbors"], std_ratio=pcd_vars["std_dev"]
    )
    pcd_combined, _ = pcd_combined.remove_radius_outlier(
        nb_points=pcd_vars["radius_pnts"], radius=pcd_vars["radius"]
    )

    if vis_on:
        _visualize_pcd(
            pcd_combined,
            folders[3].format(rob_num),
            filenames[7],
        )

    o3d.io.write_point_cloud(
        _create_file_path(
            folder=folders[3].format(rob_num), filename=filenames[8].format(rob_num)
        ).__str__(),
        pcd_combined,
    )


def stitch_shed(
    rob_nums, transform=False, stitch=False, stitch_full=False, pose_range=False, vis_on=False
):
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

    if transform:
        _transform_pointclouds(rob_nums, pose_range, folders, filenames)

    if stitch:
        _stitch_pcd(rob_nums, pose_range, folders, filenames, vis_on)

    if stitch_full:
        _stitch_full(folders, filenames, vis_on)


def stitch_ECL_demo(
    rob_nums, transform=False, stitch=False, stitch_full=False, pose_range=False, vis_on=False
):
    folders = [
        "transformations",
        "configs/stitch_ECL_demo/R{}",
        "data/stitch_ECL_demo/R{}",
        "data/stitch_ECL_demo",
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

    if transform:
        _transform_pointclouds(rob_nums, pose_range, folders, filenames)

    if stitch:
        _stitch_pcd(rob_nums, pose_range, folders, filenames, vis_on)

    if stitch_full:
        _stitch_full(folders, filenames, vis_on)


if __name__ == "__main__":
    rob_nums = [1, 2]

    stitch_shed(
        rob_nums,
        transform=False,
        stitch=False,
        stitch_full=True,
        pose_range=range(1, 8),
        vis_on=False,
    )

    # stitch_ECL_demo(rob_nums, transform=True, stitch=True, stitch_full=True, vis_on=True)
