import numpy as np
import cv2 as cv


from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Transformation

from communication.communication import ABBCommunication

# LOCAL IMPORTS
from src.camera.use import capture_image
from src.camera.convert import convert2png, load_pointcloud
from src.utility.io import file_name

from calibration_planes import f1,a1, points, x_vectors, y_vectors
from calibration_planes import make_frames, make_yaml_calibration, read_yaml 



import time




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

def move_to_frame(i):
    if robot.state == 1:
        input("Press Enter to move to pos...")
        print("moving to pos:", i)

        robot.float_arbitrary = int(1)
        robot.set_wobj_to_num(int(2))      
        robot.set_speed_to_num(2)  

        pose_cart_base = robot.get_current_pose_cartesian_base()
        make_yaml(99,pose_cart_base)

        f1 = frames[i]
        robot.send_pose_cartesian(input = f1, ext_axes_in = a1)

        input("Press Enter to take photo...\n")
        pose_cart_base = robot.get_current_pose_cartesian_base()
        make_yaml_calibration(i+1,pose_cart_base)

        name = "img{:02d}".format(i+1)
        capture_image(
            folder = "dataset",
            output_file = file_name(name, ".zdf"),
            setting_file= "detection_settings.yml",
            )

        pc = load_pointcloud(
            folder = "dataset",
            input_file = file_name(name, ".zdf")
        )

        img_png = convert2png(
            pointcloud = pc,
            folder = "dataset_img",
            output_file = file_name(name, "_rgb.png"),
            )

        return i + 1
    else:
        pass


def pickup():

    # robot.float_arbitrary = int(1)
    # robot.set_wobj_to_num(int(2))      
    # robot.set_speed_to_num(2)  

    H2_tcp_cam = read_yaml("transformations","H2_tcp_cam.yaml")
    H3_base_tcp = read_yaml("transformations","H3_base_tcp.yaml")


    #object to camera
    t1 = np.array([
        [-1.8410609057830513e-04, -9.2566724562835423e-01, 3.7833862672976393e-01, -5.8480737304687500e+02],
        [-9.9999826711914852e-01, 8.7130398488241838e-04, 1.6451711357807401e-03, -1.5608366699218750e+03],
        [-1.8525289869500139e-03, -3.7833766822797615e-01, -9.2566580196967985e-01, 1.6019981689453125e+03],
        [0., 0., 0., 1. ],
    ])


    #Make into COMPAS objects
    H2 = Transformation.from_matrix(H2_tcp_cam.tolist())
    H3 = Transformation.from_matrix(H3_base_tcp.tolist())

    print(H2)

    #robot.send_pose_cartesian(input = ff, ext_axes_in = a1)



# ip_abb = '192.168.125.1'
# robot = ABBCommunication("ABB", ip_abb, port_snd=30003, port_rcv=30004)
# robot.start()

pickup()

i = 1
# while robot.running:
#     try:
#         #i = move_to_frame(i)

#         pickup()

#         #move_to_preset("ECL_parking_mid")
#         #move_to_preset("ECL_camera_attach")
#     except:
#         robot.close()

