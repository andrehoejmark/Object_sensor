
"""
Reads LIDAR point cloud data [3D-coordinates) from serial port (connected to Arduino),
projects these to the 2D camera-plane, and passes them to the object-detection script
through a callback.
"""

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv

# built-in modules
import os
import sys
import serial
import string
import time
from threading import Thread

# custom modules
import calibrate

num_points_per_batch = 1000  # project and send this many points at a time
inches_per_meter = 39.3700787
pixels_per_inches = 96  # usually between 72-130; check with the used camera

focal_length = None  # focal length of the used camera
coords = []  # the 3D LIDAR coordinates received from Arduino over serial port
distances = []  # the distances associated with each 3D point
ser = serial.Serial('/dev/ttyACM0', 9600, 8, 'N', 1, timeout=1)


def retrieve_object_coords(coords_string):
    # retrieve (parse) the 3D coordinates
    # (might have to convert coordinates from LIDAR
    # from meter units to pixel units through camera's DPI - dots (pixels) per inch)
    x, y, z, distance = coords_string.split(' ')
    return [x, y, z, distance]


def run(callback):
    global focal_length

    # retrieve camera parameters
    camera_matrix, dist_coefs, rvecs, tvecs = calibrate.calibrate_camera()

    # extract focal length of camera (there is one for x and one for y; take average)
    focal_length = (camera_matrix[0, 0] + camera_matrix[1, 1]) * 0.5

    # read LIDAR data 3D coordinates from Arduino (over serial port)
    while True:
        coords_string = " "
        while coords_string != "":
            # read encoded 3D coordinate over serial port, format: (x, y, z, distance)
            coords_string = ser.readline()
            # retrieve the 3D coordinate
            object_point = retrieve_object_coords(coords_string)
            # store the retrieved 3D coordinate
            coords.append(object_point[:3])
            distances.append(object_point[3])

            # project 3D points to the camera's 2D plane when enough points are retrieved
            if len(coords) >= num_points_per_batch:
                # cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs[, imagePoints[, jacobian[, aspectRatio]]]) → imagePoints, jacobian
                imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, camera_matrix, dist_coefs)
                #print('Projected LIDAR points to camera view: {}'.format(imagePoints))
                # pass the points to the object-detection script for matching to its detected bounding boxes
                t = Thread(target=callback, args=(imagePoints, distances))
                # run in background
                t.daemon = True
                t.start()
                coords.clear()
                distances.clear()
