from communication.communication import ABBCommunication

# LOCAL IMPORTS
from src.camera.use import capture_image
from src.utility.io import file_name

from calibration_planes import f1,a1, points, x_vectors, y_vectors
from calibration_planes import make_frames, make_yaml

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
            "ECL_parking_low"  : [angle_1, -66, 63, 0.0, 90, -90, track_x, 0, 0],
            "ECL_camera_attach"  : [angle_1, -10, 55, 91, 90, 133, track_x, 0, 0],
        }



        robot.send_axes_absolute(joint_presets[joints_abb], int_arr = None)
    else:
        pass

frames = make_frames(points, x_vectors, y_vectors)

def move_to_frame():
    if robot.state == 1:
        #rob_num = int(input("\nSet rob: "))
        rob_num = 1
        frame_num = int(input("\nSet frame: "))

        robot.float_arbitrary = int(rob_num)
        robot.set_wobj_to_num(int(2))      
        robot.set_speed_to_num(2)  

        f1 = frames[frame_num]
        robot.send_pose_cartesian(input = f1, ext_axes_in = a1)

        take_image = int(input("\nTake Image: "))

        if take_image:
            pose_cart_base = robot.get_current_pose_cartesian_base()
            make_yaml(frame_num+1,pose_cart_base)

            name = "img{:02d}.zdf".format(frame_num+1)
            capture_image(
                folder = "dataset",
                output_file = name,
                )
    else:
        pass


while robot.running:
    try:
        move_to_frame()
        #move_to_preset("ECL_parking_mid")
        #move_to_preset("ECL_camera_attach")
    except:
        robot.close()

