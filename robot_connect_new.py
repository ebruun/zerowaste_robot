# Before running this example, make sure to run
# "docker compose up" on the docker/moveit folder
import compas_rrc as rrc

from compas_fab.backends import RosClient
from compas_fab.robots import to_radians

from compas.robots import Configuration
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector

import math

# LOCAL IMPORTS
from calibration_planes import make_frames
from calibration_planes import a1, points, x_vectors, y_vectors #this is dumb, fix later (make a .json import)

def move_by_frame(abb, speed):
    
    abb.send(rrc.PrintText('Move by frame'))
    frame = abb.send_and_wait(rrc.GetFrame(), timeout=10)

    # Change in Global Z axis (in millimeters)
    frame.point[2] += 100

    # Move robot the new pos
    return abb.send_and_wait(rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR),timeout=10)

def move_by_robtarget(abb, speed):
    abb.send(rrc.PrintText('Move by rob target'))
    frame, external_axes = abb.send_and_wait(rrc.GetRobtarget())

    # Change X value of the frame
    frame.point[0] -= 50

    # Move track forward
    external_axes[0] += 200

    # Move robot the new pos
    return abb.send_and_wait(rrc.MoveToRobtarget(frame, external_axes, speed, rrc.Zone.FINE),timeout=10)

def move_by_joints(abb, speed):
    abb.send(rrc.PrintText('Move by joints'))

    joint_presets = {
    "ECL_parking_high": ([-90, -21, 46, 0.0, 66, -90.],[3900]),
    "ECL_parking_mid": ([-90, -46, 44, 0.0, 90, -90.],[3900]),
    "ECL_parking_low": ([-90, -66, 63, 0.0, 90, -90.],[3900]),
    "zero_position": ([0, 0, 0, 0, 0, 0.],[3900.]),
    "camera_attach": ([-90, -10, 55, 91, 90, 133.], [3900]),
    }

    robot_joints, external_axes = joint_presets["camera_attach"]

    # Move robot to start position
    return abb.send_and_wait(rrc.MoveToJoints(robot_joints, external_axes, speed, rrc.Zone.FINE),timeout=10)    


# Load robot without geometry

def robot_connect(rob_num):

    #with RosClient('localhost') as ros:

    ros = rrc.RosClient('localhost')
    ros.run()

    # set Robot to connect to
    abb = rrc.AbbClient(ros, rob_num)
    abb.send(rrc.PrintText("CONNECTING TO {}".format(rob_num)))
    abb.send(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    # Set work object
    abb.send(rrc.SetWorkObject('rob1_acadia_EBIT_01'))
    #abb.send(rrc.SetWorkObject('ECL_platform'))
    #abb.send(rrc.SetWorkObject('wobj0'))

    # Set speed [mm/s]
    speed = 100

    # Execute command
    abb.send(rrc.PrintText('Starting...'))

    #done = move_by_frame(abb,speed)
    #done = move_by_robtarget(abb,speed)
    #done = move_by_joints(abb,speed)

    # # Print feedback
    # print('Feedback = ', done)

    # # End of Code
    # print('Finished')

    # # Close client
    # ros.close()
    # ros.terminate()
    return ros, abb


# # Move robot to a plane in saved list (for calibration)
# def move_to_frame(i):
#     if robot.state == 1:
#         input("Press Enter to move to pos...")
#         print("moving to pos:", i)

#         robot.float_arbitrary = int(1)
#         robot.set_wobj_to_num(int(2))
#         robot.set_speed_to_num(2)

#         f = frames[i]
#         robot.send_pose_cartesian(input=f, ext_axes_in=a1)

def _create_current_configuration(rob_num,robot,abb):

    robot_joints, external_axes = _get_current_joints_axis(abb)

    robot_joints = rrc.RobotJoints(to_radians(robot_joints.values)) #in radians
    external_axes = external_axes[0]/1000 # in m

    #for use in path planning
    #radians and meters
    config_scaled = robot.zero_configuration()

    if rob_num == '/rob1':
        config_scaled['r1_cart_joint'] = external_axes
        config_scaled['r1_joint_1'] = robot_joints[0]
        config_scaled['r1_joint_2'] = robot_joints[1]
        config_scaled['r1_joint_3'] = robot_joints[2]
        config_scaled['r1_joint_4'] = robot_joints[3]
        config_scaled['r1_joint_5'] = robot_joints[4]
        config_scaled['r1_joint_6'] = robot_joints[5]
    elif rob_num == '/rob2':
        config_scaled['r2_cart_joint'] = external_axes
        config_scaled['r2_joint_1'] = robot_joints[0]
        config_scaled['r2_joint_2'] = robot_joints[1]
        config_scaled['r2_joint_3'] = robot_joints[2]
        config_scaled['r2_joint_4'] = robot_joints[3]
        config_scaled['r2_joint_5'] = robot_joints[4]
        config_scaled['r2_joint_6'] = robot_joints[5]
        
    return config_scaled

def _get_current_joints_axis(abb):
    # return list of joint and track values
    return abb.send_and_wait(rrc.GetJoints(), timeout=10)



def calibration(rob_num,frames):

    ros, abb = robot_connect(rob_num)
    robot = ros.load_robot()

    for frame in frames[0:1]:
        # frame = abb.send_and_wait(rrc.GetFrame(), timeout=10)
        # print (frame)

        frame = Frame(
            Point(-0.350, 0.601, -0.118),
            Vector(1, 0, 0.),
            Vector(0, -1, 0.),
            )

        print (frame)

        start_config = _create_current_configuration(rob_num, robot, abb)
        print (start_config)

        end_config = robot.inverse_kinematics(frame,start_config)
        print (end_config)






if __name__ == "__main__":
    #ros, abb = robot_connect('/rob1')

    frames = make_frames(points, x_vectors, y_vectors)

    calibration('/rob1', frames)

    