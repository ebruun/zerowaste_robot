from communication.communication import ABBCommunication

from planes import f1,a1

import time

ip_abb = '192.168.125.1'
robot = ABBCommunication("ABB", ip_abb, port_snd=30003, port_rcv=30004)
robot.start()


def move_to_preset(joints_abb):
    if robot.state == 1:
        rob_num = int(input("\nSet rob: "))

        robot.float_arbitrary = int(rob_num)
        robot.set_wobj_to_num(int(2))
        robot.set_speed_to_num(1)

        if rob_num == 1:
            print ("USING ROBOT 1")
            track_x = 3900 #track for the preset parking values
            angle_1 = -90 
        elif rob_num == 2:
            print ("USING ROBOT 2")
            track_x = 0 #track for the preset parking values
            angle_1 = 90

        joint_presets = {
            "ECL_parking_high" : [angle_1, -21, 46, 0.0, 66, -90, track_x, 0, 0],
            "ECL_parking_mid"  : [angle_1, -46, 44, 0.0, 90, -90, track_x, 0, 0],
            "ECL_parking_low"  : [angle_1, -66, 63, 0.0, 90, -90, track_x, 0, 0]
        }

        robot.send_axes_absolute(joint_presets[joints_abb], int_arr = None)
    else:
        pass


def move_to_frame():
    if robot.state == 1:
        rob_num = int(input("\nSet rob: "))

        pose_cart = robot.get_current_pose_cartesian()
        print(pose_cart)

        robot.float_arbitrary = int(rob_num)
        robot.set_wobj_to_num(int(2))      
        robot.set_speed_to_num(1)  

        print(robot.int_speed)

        robot.send_pose_cartesian(input = f1, ext_axes_in= a1)
    else:
        pass


while robot.running:
    try:
        #move_to_frame()
        move_to_preset("ECL_parking_mid")
    except:
        robot.close()

