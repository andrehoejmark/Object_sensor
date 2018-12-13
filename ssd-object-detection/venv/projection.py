
"""
Reads LIDAR point cloud data over serial port, projects these to the 2D camera-plane,
and passes them to the object-detection script.
"""

# import the necessary packages
import numpy as np
import cv2
import pandas as pd
import os
import sys
import serial
import string
import time
import copy
from math import *
from threading import Thread
from threading import Lock
import calibrate

# global variables and constants
num_points_per_batch = 140  # project and send this many points at a time
inches_per_meter = 39.3700787
pixels_per_inches = 96.0  # usually between 72-130; check with the used camera

fov = 62.2  # horizontal fov of camera
image_width = 300.0
image_height = 300.0
dist_lidar_to_cam_meter = 0.004  # 4 cm between camera lens center to LIDAR laser output (LIDAR below camera)
y_image = dist_lidar_to_cam_meter * inches_per_meter * pixels_per_inches  # the constant y in the images
y_image += image_height / 2.0  # center of image is in origo
#focal_length = 0.0360 * inches_per_meter * pixels_per_inches  # focal length of camera (3.60 mm here)
offset_angle = 21.0  # angle offset between LIDAR 0° and camera right-side-fov angle relative to LIDAR 0°

ser = None  # the serial port

positions = []  # the angles associated with each LIDAR point
distances = []  # the distances associated with each LIDAR point

is_3d = False  # indicates whether LIDAR got y-movement or not
test_with_fake_data = False  # indicates whether to produce fake LIDAR data or not
test_with_test_data = True  # indicates whether to test with real LIDAR test data or not

fake_data = None
fake_data_mutex = Lock()


def produce_fake_data_threaded():
    # produces fake LIDAR data in the background
    project_thread = Thread(target=produce_fake_lidar_data)
    project_thread.daemon = True
    project_thread.start()


def produce_fake_lidar_data():
    # used for generating fake LIDAR data continuously for testing purposes
    global fake_data, fake_data_mutex
    fake_angle, fake_distance = 0.0, 0.0
    while True:
        fake_data_mutex.acquire()
        fake_data = "{} {}".format(fake_angle, fake_distance)
        fake_data_mutex.release()
        fake_angle = (fake_angle + 0.5) % 180.0
        fake_distance = np.random.uniform(0.0, 100.0)
        time.sleep(0.01)  # simulate delay


def read_fake_data_blocking():
    # blocks until fake LIDAR data has been produced by produce_fake_data_threaded
    global fake_data_mutex, fake_data
    while True:
        fake_data_mutex.acquire()
        if fake_data is not None:
            break
    fake_data_copy = copy.deepcopy(fake_data)
    fake_data_mutex.release()
    angle, distance = fake_data_copy.split(" ")
    angle, distance = float(angle), float(distance)
    return angle, distance


def compute_object_coord(angle_x, angle_y, distance):
    # computes and returns the 3D coordinate corresponding
    # to the passed angles and distance
    angle_x, angle_y = angle_x * pi / 180.0, angle_y * pi / 180.0
    if angle_y != 0.0:
        # 3D polar coordinates
        x = distance * cos(angle_x) * sin(angle_y)
        z = distance * sin(angle_x) * sin(angle_y)
        y = distance * cos(angle_y) + y_image
    else:
        # 2D polar coordinates
        x = distance * cos(angle_x)
        z = distance * sin(angle_x)
        y = y_image
    return x, y, z


def calc_image_point(angle):
    # set 0 angle at center of image since angle is 0-180 left to right
    x_image = image_width - (((angle - offset_angle) / fov) * image_width)
    return int(x_image), int(y_image)


def get_lidar_input():
    # returns either fake or real LIDAR data
    # is_3D indicates if LIDAR moves in y-axis or not
    global ser
    if test_with_fake_data:
        return read_fake_data_blocking()
    else:
        if ser is None:
            ser = serial.Serial('/dev/tty.usbmodem141201',
                                baudrate=9600)  # the serial port over which to receive LIDAR data
        # parse input from Arduino
        angle_x = str(ser.readline())
        is_flipped = 'x' != angle_x[0]
        distance = str(ser.readline())
        if is_3d:
            # TODO: handle if angle_y is "flipped" with either angle_x or distance
            angle_y = str(ser.readline())
            angle_y = angle_y[1: -2]
            #print("angle_y: " + str(angle_y))
        angle_x, distance = angle_x[1: -2], distance[1: -2]
        if is_flipped:
            tmp = angle_x
            angle_x = distance
            distance = tmp
        #print("angle_x: " + str(angle_x))
        #print("distance: " + str(distance))
        if is_3d:
            return float(angle_x), float(distance), float(angle_y)
        else:
            return float(angle_x), float(distance)


def get_lidar_test_data():
    angles = pd.read_csv("data/x1.csv", header=None, index_col=None)
    angles = [float(val[0]) for val in angles.values]
    dists = pd.read_csv("data/y1.csv", header=None, index_col=None)
    dists = [float(val[0]) for val in dists.values]
    #print("Angles: \n" + str(angles))
    #print("Distances: \n" + str(distances))
    return angles, dists


def run(object_detector):
    global focal_length, test_with_fake_data, distances, positions, num_points_per_batch

    # retrieve camera parameters
    camera_matrix, dist_coefs, rvecs, tvecs = calibrate.calibrate_camera()
    # compute rotation and translation matrix
    #rvecs = cv2.Rodrigues(rvecs)
    #tvecs = cv2.Rodrigues(tvecs)

    # extract focal length of camera (in pixels)
    focal_length = (camera_matrix[0, 0] + camera_matrix[1, 1]) * 0.5

    # produce fake LIDAR data for testing purposes?
    if test_with_fake_data:
        produce_fake_data_threaded()
    elif test_with_test_data:
        angles, distances = get_lidar_test_data()
        #print("Angles: " + str(angles))
        #print("Distances: " + str(distances))
        positions = [calc_image_point(angle) for angle in angles]
        #positions = [compute_object_coord(angle, 0.0, distances[i]) for i, angle in enumerate(angles)]
        #print("Positions: " + str(positions))

    # read LIDAR data over serial port from Arduino
    while True:
        try:
            if test_with_test_data:  # test with real LIDAR test data?
                time.sleep(4)  # simulate delay of receiving batch of new points
            elif is_3d: # does LIDAR have y-movement?
                angle_x, distance, angle_y = get_lidar_input()
                x, y, z = compute_object_coord(angle_x, angle_y, distance)
                positions.append((x, y, z))
                distances.append(distance)
            else:  # no y-movement and no test data
                angle, distance = get_lidar_input()
                pos = calc_image_point(float(angle))
                positions.append(pos)
                distances.append(distance)
        except Exception as e:
            print(e)

        # project points to the camera's 2D view and pass them along when enough points have been received
        if len(positions) >= num_points_per_batch:
            if is_3d:
                #print('\n3D LIDAR points: {}'.format(positions))
                positions = np.array(positions)
                positions, _ = cv2.projectPoints(positions, np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]),
                                                 camera_matrix, dist_coefs)
                positions = [(point[0][0], point[0][1]) for point in positions]
                #print('\n2D camera points: {}\n'.format(positions))
                #print('\nDistance to points: {}\n'.format(distances))
            # pass the points to the object-detection script for matching to its detected bounding boxes
            object_detector.receive_projected_points(positions, distances, angles)
            if not test_with_test_data:  # don't clear static test data
                positions.clear()
                distances.clear()
