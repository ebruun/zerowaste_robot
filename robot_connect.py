import numpy as np
import cv2 as cv
import time

from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Transformation
from compas.geometry import Translation
from compas.geometry import translate_points

from communication.communication import ABBCommunication

# LOCAL IMPORTS
from src.camera.use import capture_image
from src.camera.convert import convert2png, load_pointcloud
from src.utility.io import file_name

from calibration_planes import f1,a1, points, x_vectors, y_vectors
from calibration_planes import make_frames, make_yaml_calibration, read_yaml_frames, read_yaml_transformation


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



def move_to_frame(i):
    if robot.state == 1:
        input("Press Enter to move to pos...")
        print("moving to pos:", i)

        robot.float_arbitrary = int(1)
        robot.set_wobj_to_num(int(2))      
        robot.set_speed_to_num(2)  

        f1 = frames[i]
        robot.send_pose_cartesian(input = f1, ext_axes_in = a1)

        input("Press Enter to take photo...\n")
        pose_cart = robot.get_current_pose_cartesian() #IN WORLD CARTESIAN
        make_yaml_calibration(i+1,"calibration_data",pose_cart)

        name = "img{:02d}".format(i+1)
        capture_image(
            folder = "calibration_data",
            output_file = file_name(name, ".zdf"),
            setting_file= "detection_settings.yml",
            )

        pc = load_pointcloud(
            folder = "calibration_data",
            input_file = file_name(name, ".zdf")
        )

        img_png = convert2png(
            pointcloud = pc,
            folder = "calibration_img",
            output_file = file_name(name, "_rgb.png"),
            )

        return i + 1
    else:
        pass


def pickup():
    input("start the process...")
    robot.float_arbitrary = int(1)
    robot.set_wobj_to_num(int(2))      
    robot.set_speed_to_num(2)  

    # pose_cart = robot.get_current_pose_cartesian() #IN WORLD CARTESIAN
    # make_yaml_calibration(98,"transformations",pose_cart)
    
    input("read in the transformation matrices...\n")
    H1_cam_obj = read_yaml_frames("transformations","H1_cam_obj.yaml") #what the camera sees
    H2_tcp_cam = read_yaml_transformation("transformations","H2_tcp_cam.yaml") #camera calibration
    #H3_base_tcp = read_yaml_transformation("transformations","H3_base_tcp.yaml") #IN CARTESIAN (NOT CARTESIAN_BASE)
    H3_base_tcp = read_yaml_transformation("transformations","pos98.yaml")
    H4_wobj_base = read_yaml_transformation("transformations","H4_world_base.yaml") #T_cart_to_wobj

    #Make into COMPAS objects
    F = []
    for h in H1_cam_obj:
        h = Transformation.from_matrix(h.tolist())
        f = Frame.from_transformation(h)
        F.append(f)

    H2 = Transformation.from_matrix(H2_tcp_cam.tolist())
    H3 = Transformation.from_matrix(H3_base_tcp.tolist())
    H4 = Transformation.from_matrix(H4_wobj_base.tolist())

    H = Transformation.concatenated(H3,H2)
    H = Transformation.concatenated(H4,H)

    F_cart = F[0].transformed(H)
    print(F_cart)

    s1 = -600
    s2 = -300

    z_axis = F_cart.zaxis
    
    t1 = Translation.from_vector(z_axis*s1)
    t2 = Translation.from_vector(z_axis*s2)

    F1 = F_cart.transformed(t1)
    F2 = F_cart.transformed(t2)


    #ff = Frame(Point(1642.187, 1233.030, 1423.680), Vector(0.063, -0.997, 0.054), Vector(0.006, 0.054, 0.998))
    robot.send_pose_cartesian(input = F1, ext_axes_in = a1)

    input("do next move...\n")
    robot.send_pose_cartesian(input = F2, ext_axes_in = a1)


ip_abb = '192.168.125.1'
robot = ABBCommunication("ABB", ip_abb, port_snd=30003, port_rcv=30004)
robot.start()

frames = make_frames(points, x_vectors, y_vectors)

pickup()

# i = 0
# while robot.running:
#     try:
#        # i = move_to_frame(i)

#         pickup()

#         #move_to_preset("ECL_parking_mid")
#         #move_to_preset("ECL_camera_attach")
#     except:
#         robot.close()

