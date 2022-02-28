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

        print(f)

        save_frames_as_matrix_yaml(
            [f],
            folder="transformations",
            output_file="R{}_H4_world_wobj.yaml".format(rob_num),
        )

    elif rob_num == 2:
        pass


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

            # xyz = pc.copy_data("xyz")
            # rgba = pc.copy_data("rgba")
            # display_pointcloud(xyz, rgba[:, :, 0:3])
            # input("Press Enter to close...")

            frame.save(_create_file_path(folders[0].format(rob_num), filenames[5].format(i)))

            pc.transform(T)
            frame.save(_create_file_path(folders[0].format(rob_num), filenames[4].format(i)))

            # xyz = pc.copy_data("xyz")
            # rgba = pc.copy_data("rgba")
            # display_pointcloud(xyz, rgba[:, :, 0:3])
            # input("Press Enter to close...")

            print(T2)
            print(T3)
            print(pc)

    print("\nSTITCHING DONE")


if __name__ == "__main__":
    # rob_num = 1
    # _wobj_transformation(rob_num)

    stitch(
        rob_nums=[1],
        pose_range=range(1, 6),
        folders=["data_stitch/R{}", "transformations"],
        filenames=[
            "img{:02d}.zdf",
            "R{}_H2_robot_cam.yaml",
            "R{}_H4_world_wobj.yaml",
            "pos{:02d}.yaml",
            "img_trns{:02d}.ply",
            "img_notrans{:02d}.ply",
        ],
    )
