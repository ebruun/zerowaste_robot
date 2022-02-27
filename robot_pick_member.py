# COMPAS IMPORTS
import compas_rrc as rrc
from compas.geometry import Transformation
from compas.geometry import Translation

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.io import (
    save_frame_as_matrix_yaml,
    load_as_frames_yaml,
    load_as_transformation_yaml,
)


# function to offset a frame in the z-direction
# this is because the TCP
def _offset_frame(f, offset):

    z_axis = f.zaxis
    t = Translation.from_vector(z_axis * offset)

    return f.transformed(t)


# Pickup a member seen by camera
def pickup(abb, rob_num, robot):
    abb.send(rrc.PrintText("PRESS PLAY to start member pickup process..."))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    f = abb.send_and_wait(rrc.GetFrame(), timeout=3)
    print("--frame: ", f)
    save_frame_as_matrix_yaml("transformations", "H3_wobj_tcp_R{}.yaml".format(rob_num), f)

    abb.send(rrc.PrintText("PRESS PLAY to read in transformation matrices..."))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    F_objects = load_as_frames_yaml("transformations", "H1_cam_obj_R{}.yaml".format(rob_num))
    T2 = load_as_transformation_yaml("transformations", "H2_tcp_cam_R{}.yaml".format(rob_num))
    T3 = load_as_transformation_yaml("transformations", "H3_wobj_tcp_R{}.yaml".format(rob_num))

    T = Transformation.concatenated(T3, T2)

    F_touch = F_objects[0].transformed(T)  # Just take first object seen

    F_touch = _offset_frame(F_touch, -352)
    F_far = _offset_frame(F_touch, -300)  # almost touching member

    print(F_objects)
    print(F_touch)
    print(F_far)

    # F_touch = robot.from_t0cf_to_tcf(F_touch)
    # F_far = robot.from_t0cf_to_tcf(F_far)

    speed = 100

    abb.send(rrc.PrintText("PRESS PLAY to move to member offset location"))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE), timeout=20)
    abb.send_and_wait(
        rrc.MoveToFrame(F_far, speed, rrc.Zone.FINE, rrc.Motion.LINEAR),
        timeout=20,
    )

    speed = 30

    abb.send(rrc.PrintText("PRESS PLAY to move to member pickup location"))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE), timeout=20)
    abb.send_and_wait(
        rrc.MoveToFrame(F_touch, speed, rrc.Zone.FINE, rrc.Motion.LINEAR),
        timeout=20,
    )


if __name__ == "__main__":
    rob_num = 2
    robot, abb = connect_to_robot(rob_num)

    pickup(abb, rob_num, robot)
