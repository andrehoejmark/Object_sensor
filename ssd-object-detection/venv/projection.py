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
IM_WIDTH = 1280  # 640
IM_HEIGHT = 720  # 480
fov = 62.2  # horizontal fov of camera
num_points_per_batch = 138  # send this many points at a time (one "round" across the fov)

image_point_y = IM_HEIGHT / 2.0 + 10.0  # constant
focal_length = 0.0  # focal length of camera in pixels
offset_angle = 59.0  # offset of camera fov and LIDAR starting-angle

ser = None  # the serial port

positions = []  # the image positions associated with each point
distances = []  # the distances associated with each point
angles = []  # the angles associated with each point


def calc_image_point(angle):
    # calculates and returns the image point associated with angle
    image_point_x = IM_WIDTH - (((angle - offset_angle) / fov) * IM_WIDTH)
    return int(image_point_x), int(image_point_y)


def get_lidar_input():
    # returns LIDAR data for one point
    global ser
    if ser is None:
        # initialize the serial port over which to receive LIDAR data
        ser = serial.Serial('/dev/ttyACM0', baudrate=9600)
    try:
        # read data for one point
        angle, distance = ser.readline().decode('utf8', 'ignore').rstrip().split(",")
    except ValueError as err:
        print(err)
        return 0.0, 0.0
    return float(angle), float(distance)


def run(object_detector):
    global focal_length, distances, positions, num_points_per_batch
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
            angles.append(angle)
        except Exception as err:
            print(err)

        # batch up x points
        if len(positions) >= num_points_per_batch:
            # pass the points to the object detector and then clear lists for next batch
            object_detector.receive_points(positions, distances, angles)
            positions.clear()
            distances.clear()
            angles.clear()
