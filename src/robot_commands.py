# PYTHON IMPORTS
import time

# COMPAS IMPORTS
import compas_rrc as rrc


def io_gripper(abb, i):

    name = abb._server_protocol_check["param"].name

    rob_num = int(name.split("/")[1][-1])

    if i:
        print("close gripper")
        abb.send(rrc.SetDigital("do_{}".format(rob_num), i))
    else:
        print("open gripper")
        abb.send(rrc.SetDigital("do_{}".format(rob_num), i))

    time.sleep(0.2)


def get_current_config(robot, abb, rob_num):

    config = robot.zero_configuration()
    current_joints, current_track = abb.send_and_wait(rrc.GetJoints(), timeout=3)

    if rob_num in [1, 2]:  # only R1 and R2 have an external axis
        config["r{}_cart_joint".format(rob_num)] = current_track[0]

    config["r{}_joint_1".format(rob_num)] = current_joints[0]
    config["r{}_joint_2".format(rob_num)] = current_joints[1]
    config["r{}_joint_3".format(rob_num)] = current_joints[2]
    config["r{}_joint_4".format(rob_num)] = current_joints[3]
    config["r{}_joint_5".format(rob_num)] = current_joints[4]
    config["r{}_joint_6".format(rob_num)] = current_joints[5]

    return config


def frame_to_move(abb, frame, speed=100):
    """Taking a frame and move the robot to it"""

    abb.send(rrc.PrintText("Moving to specified frame..."))

    abb.send(rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    time.sleep(0.2)


def configs_to_move(abb, rob_num, configs, speed=100):
    """Taking a list of robot configs in (deg & mm) and move the robot to them"""

    abb.send(rrc.PrintText("Moving to specified configs..."))

    for config in configs:
        if rob_num == 1:
            axis = rrc.ExternalAxes(config["r1_cart_joint"])
            joints = rrc.RobotJoints(
                config["r1_joint_1"],
                config["r1_joint_2"],
                config["r1_joint_3"],
                config["r1_joint_4"],
                config["r1_joint_5"],
                config["r1_joint_6"],
            )
        elif rob_num == 2:
            axis = rrc.ExternalAxes(config["r2_cart_joint"])
            joints = rrc.RobotJoints(
                config["r2_joint_1"],
                config["r2_joint_2"],
                config["r2_joint_3"],
                config["r2_joint_4"],
                config["r2_joint_5"],
                config["r2_joint_6"],
            )
        elif rob_num == 3:
            axis = 0
            joints = rrc.RobotJoints(
                config["r3_joint_1"],
                config["r3_joint_2"],
                config["r3_joint_3"],
                config["r3_joint_4"],
                config["r3_joint_5"],
                config["r3_joint_6"],
            )

        # abb.send_and_wait(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE), timeout=30)
        # abb.send(rrc.PrintText("MOVE TO CONFIG DONE"))

        abb.send(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE))
        time.sleep(0.2)


if __name__ == "__main__":
    pass
