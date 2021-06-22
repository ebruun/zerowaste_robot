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
[2380.843899,3816.115549,1609.472422],
[2753.358895,3816.115549,1581.481128],
[3093.64305,3816.115549,1427.343472],
[3360.359189,3816.115549,1165.78385],
[2670.418891,3670.714065,1581.481128],
[2945.778252,3470.787757,1427.343472],
[3161.606066,3314.084684,1165.78385],
[2517.876118,3601.784111,1581.481128],
[2623.235855,3278.221702,1427.343472],
[2705.817256,3024.612139,1165.78385],
[2353.939623,3635.628781,1581.481128],
[2249.095342,3311.898982,1427.343472],
[2166.917958,3058.158217,1165.78385],
[2241.166202,3759.333234,1581.481128],
[1966.125603,3558.968628,1427.343472],
[1750.547636,3401.922015,1165.78385],
[2222.589249,3925.692883,1581.481128],
[1882.305525,3925.150929,1427.343472],
[1615.589725,3924.726142,1165.78385],
]

x_vectors = [
[0,-1,0],
[0,-1,0],
[0,-1,0],
[0,-1,0],
[-0.587528,-0.809204,0],
[-0.587528,-0.809204,0],
[-0.587528,-0.809204,0],
[-0.950859,-0.309623,0],
[-0.950859,-0.309623,0],
[-0.950859,-0.309623,0],
[-0.951351,0.308108,0],
[-0.951351,0.308108,0],
[-0.951351,0.308108,0],
[-0.588816,0.808267,0],
[-0.588816,0.808267,0],
[-0.588816,0.808267,0],
[-0.001593,0.999999,0],
[-0.001593,0.999999,0],
[-0.001593,0.999999,0],
]

y_vectors = [
[-0.999112,0,0.042122],
[-0.923974,0,0.382455],
[-0.736597,0,0.676332],
[-0.459744,0,0.888051],
[-0.747684,0.54286,0.382455],
[-0.596057,0.432771,0.676332],
[-0.372027,0.270112,0.888051],
[-0.286083,0.878569,0.382455],
[-0.228067,0.7004,0.676332],
[-0.142347,0.437152,0.888051],
[0.284684,0.879024,0.382455],
[0.226951,0.700763,0.676332],
[0.141651,0.437378,0.888051],
[0.746818,0.54405,0.382455],
[0.595367,0.43372,0.676332],
[0.371596,0.270704,0.888051],
[0.923973,0.001472,0.382455],
[0.736596,0.001173,0.676332],
[0.459744,0.000732,0.888051],
]


def make_frames(p,x,y):
    frames = []
    for p,x,y in zip(points, x_vectors, y_vectors):
        p_cmps = Point(p[0], p[1], p[2])
        x_cmps = Vector(x[0], x[1], x[2])
        y_cmps = Vector(y[0], y[1], y[2])

        frames.append(Frame(p_cmps, x_cmps, y_cmps))
    
    return frames


def make_yaml_calibration(i, folder, pose = False):
    filename = "{}/pos{:02d}.yaml".format(folder,i)
    s = cv.FileStorage(filename, cv.FileStorage_WRITE)

    pose_cart_1 = (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], pose[7], pose[8], pose[9])
    F_base = Frame.from_quaternion(pose_cart_1[3:7])
    F_base.point = pose_cart_1[0:3]
    T = Transformation.from_frame(F_base)

    PoseState = np.array(T)

    s.write('PoseState', PoseState)
    s.release()


def make_yaml_frame(folder, name, frames):
    file_name = create_file_path(folder,name)
    s = cv.FileStorage(file_name, cv.FileStorage_WRITE)

    for i,f in enumerate(frames):
        T = Transformation.from_frame(f)
        PoseState = np.array(T)
        s.write('PoseState{}'.format(i), PoseState)

    s.release()

def read_yaml_frames(folder, name):
    file_name = create_file_path(folder,name)
    s = cv.FileStorage(file_name, cv.FILE_STORAGE_READ)

    i = 0
    H = []
    reading = True
    while reading:
        try:
            h = s.getNode("PoseState{}".format(i)).mat()
            
            if h[0][0]:
                H.append(h)
                i += 1

        except:
            s.release()
            reading = False
    
    return H
        
            

def read_yaml_transformation(folder, name):

    file_name = create_file_path(folder,name)
    s = cv.FileStorage(file_name, cv.FILE_STORAGE_READ)

    h = s.getNode("PoseState").mat()

    s.release()

    return h


if __name__ == "__main__":
    make_yaml_calibration(1)