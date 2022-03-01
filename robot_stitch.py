import open3d as o3d
import numpy as np

# COMPAS IMPORTS
import compas_rrc as rrc
from compas.geometry import Transformation
from compas.geometry import Frame, allclose

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.robot_commands import configs_to_move
from src.io import (
    save_frames_as_matrix_yaml,
    load_config_json,
    load_as_frames_yaml,
    load_as_transformation_yaml,
    _create_file_path,
)

from src_cam.utility.io import load_pointcloud

from sample_utils.display import display_depthmap, display_pointcloud, display_rgb


def _wobj_transformation(rob_num):

    if rob_num == 1:
        q1 = [0.999971, 0.000727707, -0.00129108, -0.00745658]
        f = Frame.from_quaternion(q1, point=[-680.624, 1567.49, 823.009])
        q2 = f.quaternion
        allclose(q1, q2, tol=1e-03)
    elif rob_num == 2:
        pass

    print(f)

    save_frames_as_matrix_yaml(
        [f],
        folder="transformations",
        output_file="R{}_H4_world_wobj.yaml".format(rob_num),
    )


def stitch(rob_nums, pose_range, folders, filenames):

    for rob_num in rob_nums:
        for i in pose_range:
            pc, frame = load_pointcloud(
                folder=folders[0].format(rob_num), input_file=filenames[0].format(i, width=3)
            )

            # # what camera sees
            # F_objects = load_as_frames_yaml(folders[1], "R{}_H1_cam_obj.yaml".format(rob_num))

            # calculated calibration between robot and camera
            T2 = load_as_transformation_yaml(folders[1], filenames[1].format(rob_num))

            # robot position in w_obj coordinates
            T3 = load_as_transformation_yaml(folders[0].format(rob_num), filenames[3].format(i))

            T4 = load_as_transformation_yaml(
                folders[1].format(rob_num), filenames[2].format(rob_num)
            )

            T_ = Transformation.concatenated(T3, T2)
            T = Transformation.concatenated(T4, T_)

            frame.save(_create_file_path(folders[0].format(rob_num), filenames[5].format(i)))

            pc.transform(T)
            frame.save(_create_file_path(folders[0].format(rob_num), filenames[4].format(i)))

    print("\nSTITCHING DONE")


def combine_pcd(folders, filenames, rob_nums, pc_range):

    point_data = []
    color_data = []
    for i in pc_range:
        pcd = o3d.io.read_point_cloud(
            _create_file_path(
                folder=folders[0].format(rob_nums[0]), filename=filenames[4].format(i)
            ).__str__()
        )

        pcd = pcd.voxel_down_sample(voxel_size=5)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=5, std_ratio=0.5)

        p = np.asarray(pcd.points)
        p_color = np.asarray(pcd.colors)

        point_data.append(p)
        color_data.append(p_color)

    points_combined = np.concatenate(point_data, axis=0)
    colors_combined = np.concatenate(color_data, axis=0)

    pcd_combined = o3d.geometry.PointCloud()
    pcd_combined.points = o3d.utility.Vector3dVector(points_combined)
    pcd_combined.colors = o3d.utility.Vector3dVector(colors_combined)

    o3d.visualization.draw_geometries(
        [pcd_combined],
        zoom=0.7,
        front=[-0.527, 0.712, 0.463],
        lookat=[378.1, 356.96, -116.4],
        up=[0.572, -0.105, 0.813],
    )

    o3d.io.write_point_cloud(
        _create_file_path(
            folder=folders[0].format(rob_nums[0]), filename="TEST_COMBO.pts"
        ).__str__(),
        pcd_combined,
    )


if __name__ == "__main__":
    # rob_num = 1
    # _wobj_transformation(rob_num)

    folders = ["data_stitch/R{}", "transformations"]
    filenames = [
        "img{:02d}.zdf",
        "R{}_H2_robot_cam.yaml",
        "R{}_H4_world_wobj.yaml",
        "pos{:02d}.yaml",
        "img_trns{:02d}.ply",
        "img_notrans{:02d}.ply",
    ]

    # stitch(
    #     rob_nums=[1],
    #     pose_range=range(1, 6),
    #     folders=folders,
    #     filenames=filenames
    # )

    combine_pcd(rob_nums=[1], folders=folders, filenames=filenames, pc_range=range(1, 6))
