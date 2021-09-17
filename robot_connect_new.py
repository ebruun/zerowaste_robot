# Before running this example, make sure to run
# "docker compose up" on the docker/moveit folder
from compas.robots.model import joint
import compas_rrc as rrc

from compas_fab.backends import RosClient
from compas_fab.robots import to_radians

from compas.robots import Configuration
from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector

import math
import os

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

def move_by_joints(abb, speed, name):
    abb.send(rrc.PrintText('Move by joints'))

    joint_presets = {
    "ECL_parking_high": ([-90, -21, 46, 0.0, 66, -90.],[3900]),
    "ECL_parking_mid": ([-90, -46, 44, 0.0, 90, -90.],[3900]),
    "ECL_parking_low": ([-90, -66, 63, 0.0, 90, -90.],[3900]),
    "zero_position": ([0, 0, 0, 0, 0, 0.],[0.]),
    "camera_attach": ([-90, -10, 55, 91, 90, 133.], [3900]),
    }

    robot_joints, external_axes = joint_presets[name]

    print(robot_joints)
    print(external_axes)

    # Move robot to specified position
    return abb.send_and_wait(rrc.MoveToJoints(robot_joints, external_axes, speed, rrc.Zone.FINE),timeout=10)    


# Load robot without geometry

def robot_connect(rob_num):

    #with RosClient('localhost') as ros:

    ros = rrc.RosClient('localhost')
    ros.run()

    # set Robot to connect to
    abb = rrc.AbbClient(ros, rob_num)
    abb.send(rrc.PrintText("CONNECTING TO {} press play to continue".format(rob_num)))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    # Set work object
    abb.send(rrc.SetWorkObject('rob1_acadia_EBIT_01'))
    #abb.send(rrc.SetWorkObject('ECL_platform'))
    #abb.send(rrc.SetWorkObject('wobj0'))

    # Execute command
    abb.send(rrc.PrintText('Starting...'))

    #done = move_by_frame(abb,speed)
    #done = move_by_robtarget(abb,speed)
    done = move_by_joints(abb,10,"zero_position")

    return ros, abb


def _from_move_to_plan(rob_num,robot_pos,config):
    '''Taking a robot position in (deg & mm) and turning it into Configuration (rad & m)'''

    robot_joints, external_axes = robot_pos

    robot_joints = to_radians(robot_joints.values) #in radians
    external_axes = external_axes[0]/1000 # in m

    if rob_num == '/rob1':
        config['r1_cart_joint'] = external_axes
        config['r1_joint_1'] = robot_joints[0]
        config['r1_joint_2'] = robot_joints[1]
        config['r1_joint_3'] = robot_joints[2]
        config['r1_joint_4'] = robot_joints[3]
        config['r1_joint_5'] = robot_joints[4]
        config['r1_joint_6'] = robot_joints[5]
    elif rob_num == '/rob2':
        config['r2_cart_joint'] = external_axes
        config['r2_joint_1'] = robot_joints[0]
        config['r2_joint_2'] = robot_joints[1]
        config['r2_joint_3'] = robot_joints[2]
        config['r2_joint_4'] = robot_joints[3]
        config['r2_joint_5'] = robot_joints[4]
        config['r2_joint_6'] = robot_joints[5]
        
    return config

def _from_plan_to_move(rob_num,config):
    '''Taking a config in (rad & m) and turning it into JointAxes (deg) and ExternalAxes (mm)'''

    scale = 1000

    if rob_num == '/rob1':
        axis = rrc.ExternalAxes(config['r1_cart_joint']*scale)
        joints = rrc.RobotJoints(
            math.degrees(config['r1_joint_1']),
            math.degrees(config['r1_joint_2']),
            math.degrees(config['r1_joint_3']),
            math.degrees(config['r1_joint_4']),
            math.degrees(config['r1_joint_5']),
            math.degrees(config['r1_joint_6']),
        )
    elif rob_num == '/rob2':
        axis = rrc.ExternalAxes(config['r2_cart_joint']*scale)
        joints = rrc.RobotJoints(
            math.degrees(config['r2_joint_1']),
            math.degrees(config['r2_joint_2']),
            math.degrees(config['r2_joint_3']),
            math.degrees(config['r2_joint_4']),
            math.degrees(config['r2_joint_5']),
            math.degrees(config['r2_joint_6']),
        )

    return joints, axis

def _get_current_joints_axis(abb):
    # return list of joint and track values
    return abb.send_and_wait(rrc.GetJoints(), timeout=10)


def _get_current_configuration(abb,rob_num,robot):
    '''gets the current configuration'''
    robot_pos = _get_current_joints_axis(abb)

    #config to for use in path planning (rad & m)
    return _from_move_to_plan(rob_num,robot_pos,robot.zero_configuration())


def _scale_frame(f, scale):
    f.point.x *= scale
    f.point.y *= scale
    f.point.z *= scale

    return f

def plan_to_frame(abb,rob_num,robot,planning_group,f,speed):
    '''plan inverse kinematic from current frame to specified frame'''

    # current configuration
    start_config = _get_current_configuration(abb,rob_num,robot)
    #print('--start config:', start_config)

    end_config = robot.inverse_kinematics(f,start_config,group=planning_group)
    #print('--end_config:', end_config)

    joints,axis = _from_plan_to_move(rob_num,end_config)

    abb.send(rrc.PrintText("moving to {}".format(joints)))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    abb.send_and_wait(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE))


def move_to_frame(abb,f,ext,speed):
    abb.send(rrc.PrintText("press play to move to next frame..."))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    f = _scale_frame(f,1000)

    abb.send_and_wait(rrc.MoveToRobtarget(f, ext, speed, rrc.Zone.FINE))


def calibration(rob_num,planning_group,folder,num_frames,external_axis,speed):

    ros, abb = robot_connect(rob_num)
    robot = ros.load_robot()

    for i in range(num_frames):
        print('moving to frame {}'.format(i))
    
        file_path = os.path.join(os.getcwd(), folder, 'frame_{}.json'.format(i))

        #frame in 'm'
        frame = Frame.from_json(file_path)
        print('--frame:{}'.format(frame))

        #move_to_frame(abb,frame,external_axis,speed)
        plan_to_frame(abb,rob_num,robot,planning_group,frame,speed)


if __name__ == "__main__":
    # # Set speed [mm/s]
    # speed = 100

    # # calibration variables
    # calibration_folder = 'calibration_frames'
    # num_calibration_frames = 20
    # external_axis = [2000]

    # calibration('/rob1', 'robot1_axis_gripper', calibration_folder, num_calibration_frames, external_axis, speed)

    robot_connect('/rob1')

    