# PYTHON IMPORTS
import time

# COMPAS IMPORTS
import compas_rrc as rrc
from compas.geometry import Transformation
from compas.geometry import Translation

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robots
from src.robot_commands import frame_to_move
from src.io import (
    save_frames_as_matrix_yaml,
    load_as_frames_yaml,
    load_as_transformation_yaml,
)


def _io_gripper(abb, i):

    name = abb._server_protocol_check["param"].name

    rob_num = int(name.split("/")[1][-1])

    if i:
        print("close gripper")
        abb.send(rrc.SetDigital("do_{}".format(rob_num), i))
    else:
        print("open gripper")
        abb.send(rrc.SetDigital("do_{}".format(rob_num), i))

    time.sleep(0.2)


def _offset_frame(f, offset):
    """offset a frame in the z-direction"""

    z_axis = f.zaxis
    t = Translation.from_vector(z_axis * offset)

    return f.transformed(t)


def _transform_pickup_frames(abbs, rob_nums, folders, filenames):
    """
    tranform member pickup frames seen by camera to robot coordinate system
    returns two frames: far from member, on member
    """

    F = []
    R = []

    for abb, rob_num in zip(abbs, rob_nums):

        abb.send(rrc.PrintText("PRESS PLAY to start member pickup process..."))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        f = [abb.send_and_wait(rrc.GetFrame(), timeout=3)]
        save_frames_as_matrix_yaml(
            frames=f, folder=folders[0], output_file=filenames[2].format(rob_num)
        )

        abb.send(rrc.PrintText("PRESS PLAY to read in transformation matrices..."))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        # what camera sees
        F_objects = load_as_frames_yaml(folder=folders[0], name=filenames[0].format(rob_num))

        # transformation matrix between TOOL0 and CAMERA
        T2 = load_as_transformation_yaml(folder=folders[0], name=filenames[1].format(rob_num))

        # transformation matrix between ROBOT BASE (WOBJ) and TOOL0
        T3 = load_as_transformation_yaml(folder=folders[0], name=filenames[2].format(rob_num))

        T = Transformation.concatenated(T3, T2)

        F_touch = F_objects[0].transformed(T)  # Just take first object seen
        F_far = _offset_frame(F_touch, -300)  # almost touching member
        F_touch = _offset_frame(F_touch, 50)

        F.append([F_far, F_touch])
        R.append(abb)

    return F, R


def pick_ECL_demo(abbs, rob_nums):
    """Pickup single members"""

    folders = ["transformations"]
    filenames = ["R{}_H1_cam_obj.yaml", "R{}_H2_tool0_cam.yaml", "R{}_H3_rbase_tool0.yaml"]

    F, R = _transform_pickup_frames(abbs, rob_nums, folders, filenames)

    abbs[0].send(rrc.PrintText("PRESS PLAY to move to member offset location"))
    abbs[0].send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE), timeout=20)

    for abb, f in zip(R, [item[0] for item in F]):
        frame_to_move(abb, f)

    abbs[0].send(rrc.PrintText("PRESS PLAY to move to member pickup location"))
    abbs[0].send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE), timeout=20)

    for abb, f in zip(R, [item[1] for item in F]):
        frame_to_move(abb, f, speed=30)

    abbs[0].send(rrc.PrintText("PRESS PLAY to close gripper"))
    abbs[0].send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE), timeout=20)

    for abb in abbs:
        _io_gripper(abb, 1)

    abbs[0].send(rrc.PrintText("PRESS PLAY to open gripper"))
    abbs[0].send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE), timeout=20)

    for abb in abbs:
        _io_gripper(abb, 0)


if __name__ == "__main__":
    rob_nums = [1, 2]
    abbs, robots = connect_to_robots(rob_nums)
    pick_ECL_demo(abbs, rob_nums)
