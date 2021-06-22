from compas.geometry import Frame
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Transformation

import cv2 as cv
import numpy as np

from pathlib import Path


from src.utility.io import create_file_path

f1 = Frame(
    Point(1231.0929857258,2419.28094095468,1301.099233685),
    Vector(0.0790099124356339,0.996507820346806,0.0270110667055376),
    Vector(0.679305952325499,-0.0339896636151278,-0.733067613459003),
    )

a1 = [3900]

f1 = Frame(
    Point(-111.056, 21.415, 854.703),
    Vector(-0.440, -0.894, -0.081),
    Vector(0.630, -0.372, 0.681),
    )

f2 = Frame(
    Point(199.158, 17.724, 900.672),
    Vector(-0.002, -0.942, 0.336),
    Vector(-0.990, -0.046, -0.135))

points = [
[2297.092749,3933.173331,1647.115011],
[2664.364259,3932.949994,1513.494414],
[2963.472071,3931.896321,1261.944925],
[3158.081047,3930.14031,923.024377],
[2589.365347,3590.179231,1515.260682],
[2831.896289,3413.943519,1264.534825],
[2989.838738,3298.65495,926.148058],
[2327.287964,3356.865891,1515.548501],
[2421.112325,3072.109583,1264.840358],
[2482.680518,2886.490624,926.46507],
[1978.138646,3322.040393,1514.248042],
[1887.871905,3036.835483,1262.744934],
[1830.133723,2850.925793,923.854443],
[1675.149854,3498.991857,1511.855549],
[1435.655114,3321.581522,1259.04815],
[1281.204753,3205.531698,919.31237],
[1533.939578,3820.197083,1509.283977],
[1237.024098,3817.690991,1255.160668],
[1045.360314,3814.993692,914.572069],
]

x_vectors = [
[-0.001614,0.99998,-0.006108],
[-0.001614,0.99998,-0.006108],
[-0.001614,0.99998,-0.006108],
[-0.001614,0.99998,-0.006108],
[0.586215,0.810152,-0.002403],
[0.586215,0.810152,-0.002403],
[0.586215,0.810152,-0.002403],
[0.95035,0.311177,0.00222],
[0.95035,0.311177,0.00222],
[0.95035,0.311177,0.00222],
[0.951839,-0.306541,0.005995],
[0.951839,-0.306541,0.005995],
[0.951839,-0.306541,0.005995],
[0.590114,-0.807285,0.007482],
[0.590114,-0.807285,0.007482],
[0.590114,-0.807285,0.007482],
[0.003207,-0.999976,0.006115],
[0.003207,-0.999976,0.006115],
[0.003207,-0.999976,0.006115],
]

y_vectors = [
[0.999284,0.001382,-0.037801],
[0.925621,-0.000818,-0.37845],
[0.73952,-0.002918,-0.673129],
[0.463587,-0.004664,-0.886039],
[0.750209,-0.543956,-0.375897],
[0.59968,-0.435911,-0.671093],
[0.376307,-0.274914,-0.884768],
[0.289156,-0.880416,-0.375842],
[0.232126,-0.704138,-0.671049],
[0.146899,-0.442328,-0.884741],
[-0.281605,-0.881807,-0.378306],
[-0.222887,-0.705247,-0.673014],
[-0.137095,-0.44302,-0.885968],
[-0.744275,-0.547598,-0.38235],
[-0.59173,-0.438814,-0.676238],
[-0.367307,-0.276727,-0.88798],
[-0.922303,-0.005321,-0.38643],
[-0.733656,-0.006508,-0.67949],
[-0.455889,-0.006904,-0.89001],
]


def make_frames(p,x,y):
    frames = []
    for p,x,y in zip(points, x_vectors, y_vectors):
        p_cmps = Point(p[0], p[1], p[2])
        x_cmps = Vector(x[0], x[1], x[2])
        y_cmps = Vector(y[0], y[1], y[2])

        frames.append(Frame(p_cmps, x_cmps, y_cmps))
    
    return frames


def make_yaml_calibration(i, pose = False):
    filename = "dataset/pos{:02d}.yaml".format(i)
    s = cv.FileStorage(filename, cv.FileStorage_WRITE)

    pose_cart_1 = (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], pose[7], pose[8], pose[9])
    F_base = Frame.from_quaternion(pose_cart_1[3:7])
    F_base.point = pose_cart_1[0:3]
    T = Transformation.from_frame(F_base)

    PoseState = np.array(T)

    s.write('PoseState', PoseState)

    pass

def make_yaml_frame(folder, name, f):
    file_name = create_file_path(folder,name)
    s = cv.FileStorage(file_name, cv.FileStorage_WRITE)

    T = Transformation.from_frame(f)
    PoseState = np.array(T)
    s.write('PoseState', PoseState)

def read_yaml(folder, name):

    file_name = create_file_path(folder,name)
    file_storage = cv.FileStorage(file_name, cv.FILE_STORAGE_READ)

    h = file_storage.getNode("PoseState").mat()

    file_storage.release()

    return h


if __name__ == "__main__":
    make_yaml_calibration(1)