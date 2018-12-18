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
num_points_per_batch = 180  # send this many points at a time
inches_per_meter = 39.3700787
inches_per_cm = inches_per_meter / 100.0
pixels_per_inches = 96.0
fov = 62.2  # horizon§tal fov of camera
fov_half = fov / 2.0
image_width = 1280  # 480.0
image_height = 720  # 640.0

dist_lidar_to_cam_meter = 0.002  # +2 cm between camera lens center to LIDAR laser output
image_point_y = image_height / 2.0 + dist_lidar_to_cam_meter * inches_per_meter * pixels_per_inches  # constant
focal_length = 0.0
offset_angle = 0.0

ser = None  # the serial port

positions = []  # the angles associated with each point
distances = []  # the distances associated with each point


def calc_image_point(angle):
    # set 0 angle at center of image since angle is 0-180 left to right
    image_point_x = image_width - (((angle - offset_angle) / fov) * image_width)
    return int(image_point_x), int(image_point_y)


def get_lidar_input():
    # returns LIDAR data
    global ser
    if ser is None:
        # the serial port over which to receive LIDAR data
        ser = serial.Serial('/dev/ttyACM0', baudrate=9600)
    try:
        angle, distance = ser.readline().decode('utf8', 'ignore').rstrip().split(",")
    except ValueError as err:
        print(err)
        return 0.0, 0.0
    return float(angle), float(distance)


def run(object_detector):
    global focal_length, test_with_fake_data, distances, positions, num_points_per_batch
    # retrieve camera parameters
    camera_matrix, dist_coefs, rvecs, tvecs = calibrate.calibrate_camera()
    # extract focal length of camera (in pixels)
    focal_length = (camera_matrix[0, 0] + camera_matrix[1, 1]) * 0.5

    # read LIDAR data over serial port from Arduino
    while True:
        try:
            # read data
            angle, distance = get_lidar_input()
            # compute image points from angles
            x, y = calc_image_point(float(angle))
            positions.append((x, y))
            distances.append(distance)
        except Exception as err:
            print(err)

        # batch up x points
        if len(positions) >= num_points_per_batch:
            # pass the points to the object detector
            object_detector.receive_points(positions, distances)
            positions.clear()
            distances.clear()

