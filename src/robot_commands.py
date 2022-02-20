# this file is just a random collection of robot control functions, need to clean up later

# PYTHON IMPORTS
# PYTHON IMPORTS
import math

# COMPAS IMPORTS
import compas_rrc as rrc
from compas_fab.robots import to_radians

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.io import load_config_json


def move_by_frame(abb, speed):

    abb.send(rrc.PrintText("Move by frame"))
    frame = abb.send_and_wait(rrc.GetFrame(), timeout=10)

    # Change in Global Z axis (in millimeters)
    frame.point[2] += 100

    # Move robot the new pos
    return abb.send_and_wait(
        rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR),
        timeout=10,
    )


def move_by_robtarget(abb, speed):
    abb.send(rrc.PrintText("Move by rob target"))
    frame, external_axes = abb.send_and_wait(rrc.GetRobtarget())

    # Change X value of the frame
    frame.point[0] -= 50

    # Move track forward
    external_axes[0] += 200

    # Move robot the new pos
    return abb.send_and_wait(
        rrc.MoveToRobtarget(frame, external_axes, speed, rrc.Zone.FINE),
        timeout=10,
    )


def configs_to_move(abb, rob_num, configs):
    """Taking a list of robot configs in (deg & mm) and move the robot to them"""

    abb.send(rrc.PrintText("Moving to specified configs..."))
    speed = 100

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

        abb.send_and_wait(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE), timeout=30)
        abb.send(rrc.PrintText("MOVE TO CONFIG DONE"))


def _from_move_to_plan(rob_num, robot_pos, config):
    """Taking a robot position in (deg & mm) and turning it into Configuration (rad & m)"""

    robot_joints, external_axes = robot_pos

    robot_joints = to_radians(robot_joints.values)  # in radians
    external_axes = external_axes[0] / 1000  # in m

    if rob_num == "/rob1":
        config["r1_cart_joint"] = external_axes
        config["r1_joint_1"] = robot_joints[0]
        config["r1_joint_2"] = robot_joints[1]
        config["r1_joint_3"] = robot_joints[2]
        config["r1_joint_4"] = robot_joints[3]
        config["r1_joint_5"] = robot_joints[4]
        config["r1_joint_6"] = robot_joints[5]
    elif rob_num == "/rob2":
        config["r2_cart_joint"] = external_axes
        config["r2_joint_1"] = robot_joints[0]
        config["r2_joint_2"] = robot_joints[1]
        config["r2_joint_3"] = robot_joints[2]
        config["r2_joint_4"] = robot_joints[3]
        config["r2_joint_5"] = robot_joints[4]
        config["r2_joint_6"] = robot_joints[5]

    return config


def _from_plan_to_move(rob_num, config):
    """Taking a config in (rad & m) and turning it into JointAxes (deg) and ExternalAxes (mm)"""

    scale = 1000

    if rob_num == "/rob1":
        axis = rrc.ExternalAxes(config["r1_cart_joint"] * scale)
        joints = rrc.RobotJoints(
            math.degrees(config["r1_joint_1"]),
            math.degrees(config["r1_joint_2"]),
            math.degrees(config["r1_joint_3"]),
            math.degrees(config["r1_joint_4"]),
            math.degrees(config["r1_joint_5"]),
            math.degrees(config["r1_joint_6"]),
        )
    elif rob_num == "/rob2":
        axis = rrc.ExternalAxes(config["r2_cart_joint"] * scale)
        joints = rrc.RobotJoints(
            math.degrees(config["r2_joint_1"]),
            math.degrees(config["r2_joint_2"]),
            math.degrees(config["r2_joint_3"]),
            math.degrees(config["r2_joint_4"]),
            math.degrees(config["r2_joint_5"]),
            math.degrees(config["r2_joint_6"]),
        )

    return joints, axis


def _get_current_joints_axis(abb):
    # return list of joint and track values
    return abb.send_and_wait(rrc.GetJoints(), timeout=10)


def _get_current_configuration(abb, rob_num, robot):
    """gets the current configuration"""
    robot_pos = _get_current_joints_axis(abb)

    # --config to for use in path planning (rad & m)-- #
    return _from_move_to_plan(rob_num, robot_pos, robot.zero_configuration())


def _scale_frame(f, scale):
    f.point.x *= scale
    f.point.y *= scale
    f.point.z *= scale

    return f


def plan_to_frame(abb, rob_num, robot, planning_group, f, speed):
    """plan inverse kinematic from current frame to specified frame"""

    # --current configuration --#
    start_config = _get_current_configuration(abb, rob_num, robot)
    # print('--start config:', start_config)

    end_config = robot.inverse_kinematics(f, start_config, group=planning_group)
    # print('--end_config:', end_config)

    joints, axis = _from_plan_to_move(rob_num, end_config)

    abb.send(rrc.PrintText("moving to {}".format(joints)))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    abb.send_and_wait(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE))


def move_to_frame(abb, f, ext, speed):
    abb.send(rrc.PrintText("press play to move to next frame..."))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    f = _scale_frame(f, 1000)

    abb.send_and_wait(rrc.MoveToRobtarget(f, ext, speed, rrc.Zone.FINE))


if __name__ == "__main__":
    rob_num = 2
    preset_name = [
        "camera_attach",
        "zero_position",
        "ECL_park_high",
        "ECL_park_mid",
        "ECL_park_low",
    ]

    robot, abb = connect_to_robot(rob_num)

    configs = load_config_json(
        "configs/presets/R{}",
        preset_name[2] + ".json",
        rob_num,
    )

    configs_to_move(abb, rob_num, configs)
