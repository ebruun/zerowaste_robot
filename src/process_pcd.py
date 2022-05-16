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


# def _transform_pointclouds(rob_nums, pose_range, folders, filenames):
#     """
#     transform all pointclouds for multipled robots
#     """

#     print("START POINTCLOUD TRANSFORM\n")

#     for rob_num in rob_nums:
#         print("\n#######FOR R{}#########".format(rob_num))
#         for i in pose_range:
#             print("\nLOAD AND TRANSFORM POSE {}".format(i))

#             pc, frame = load_pointcloud(
#                 folder=folders[2].format(rob_num),
#                 input_file=filenames[0].format(i, width=3),
#             )

#             # transformation matrix between TOOL0 and CAMERA
#             T2 = load_as_transformation_yaml(folders[0], filenames[2].format(rob_num))

#             # transformation matrix between ROBOT BASE (WOBJ) and TOOL0
#             T3 = load_as_transformation_yaml(folders[2].format(rob_num), filenames[3].format(i))

#             # transformation matrix between WORLD0 and ROBOT BASE (WOBJ)
#             T4 = load_as_transformation_yaml(folders[0], filenames[4].format(rob_num))

#             T = Transformation.concatenated(T4, Transformation.concatenated(T3, T2))

#             pc.transform(T)
#             frame.save(_create_file_path(folders[2].format(rob_num), filenames[1].format(i)))

#     print("\nPOINTCLOUD TRANSFORMS DONE")


def _visualize_pcd(pcd, folder, filename):
    vis_settings = load_o3d_view_settings(folder, filename)

    pcd.estimate_normals()

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


def _visualize_pcd_interactive(pcd, folder, filename):
    print("")
    print("1) Please pick at least three correspondences using [shift + left click]")
    print("   Press [shift + right click] to undo point picking")
    print("2) Afther picking points, press q for close the window")

    vis_settings = load_o3d_view_settings(folder, filename)
    pcd.estimate_normals()

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)

    ctr = vis.get_view_control()
    ctr.set_front(vis_settings["front"])
    ctr.set_up(vis_settings["up"])
    ctr.set_zoom(vis_settings["zoom"])
    ctr.set_lookat(vis_settings["lookat"])

    vis.run()  # user picks points
    vis.destroy_window()
    print("")

    return vis.get_picked_points()


def pcd_load(folders, filenames, save=False):
    pcd = o3d.io.read_point_cloud(
        _create_file_path(folder=folders[0], filename=filenames[1]).__str__()
    )

    if save:
        o3d.io.write_point_cloud(
            _create_file_path(folder=folders[0], filename=filenames[2]).__str__(),
            pcd,
        )

    return pcd


def pcd_pick_points(pcd, folders, filenames):
    pnts = _visualize_pcd_interactive(
        pcd,
        folders[1],
        filenames[0],
    )

    pnts_dict = {}

    for i, p in enumerate(pnts):

        pnts_dict[i] = {"p_xyz": list(pcd.points[p]), "n_xyz": list(pcd.normals[p])}

    return pnts_dict


def pcd_stitch_individual_rob(rob_nums, pc_range, folders, filenames, pcd_vars, vis_on=False):
    """
    combine pointclouds from all captures with a single robot
    """

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

            # reduce individual files (otherwise too slow when all together)
            pcd = pcd.voxel_down_sample(voxel_size=pcd_vars["voxels"])
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=pcd_vars["neighbors"], std_ratio=pcd_vars["std_dev"]
            )

            point_data.append(np.asarray(pcd.points))
            color_data.append(np.asarray(pcd.colors))

        points_combined = np.concatenate(point_data, axis=0) / 1000  # mm -> m
        colors_combined = np.concatenate(color_data, axis=0)

        pcd_combined = o3d.geometry.PointCloud()
        pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
        pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

        # reduce when all together
        pcd_combined = pcd_combined.voxel_down_sample(voxel_size=pcd_vars["voxels"])
        pcd_combined, _ = pcd_combined.remove_statistical_outlier(
            nb_neighbors=pcd_vars["neighbors"], std_ratio=pcd_vars["std_dev"]
        )
        # pcd_combined, _ = pcd_combined.remove_radius_outlier(
        #     nb_points=pcd_vars["radius_pnts"], radius=pcd_vars["radius"]
        # )

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


def pcd_stitch_combine_rob(folders, filenames, pcd_vars, vis_on=False):
    """
    combine pointclouds from both robots
    """

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
    # pcd_combined, _ = pcd_combined.remove_statistical_outlier(
    #     nb_neighbors=pcd_vars["neighbors"], std_ratio=pcd_vars["std_dev"]
    # )
    # pcd_combined, _ = pcd_combined.remove_radius_outlier(
    #     nb_points=pcd_vars["radius_pnts"], radius=pcd_vars["radius"]
    # )

    if vis_on:
        _visualize_pcd(
            pcd_combined,
            folders[3],
            filenames[7],
        )

    o3d.io.write_point_cloud(
        _create_file_path(
            folder=folders[3].format(rob_num), filename=filenames[8].format(rob_num)
        ).__str__(),
        pcd_combined,
    )


if __name__ == "__main__":
    rob_nums = [1]
