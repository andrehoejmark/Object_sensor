
"""
Reads LIDAR point cloud data [3D-coordinates) from serial port (connected to Arduino),
projects these to the 2D camera-plane, and passes them to the object-detection script
through a callback.
"""

import numpy as np
import cv2 as cv

# built-in modules
import os
import sys
import serial
import string
import time
from threading import Thread
from threading import Lock

# custom modules
import calibrate

# global variables and constants
num_points_per_batch = 360  # project and send this many points at a time
inches_per_meter = 39.3700787
pixels_per_inches = 96  # usually between 72-130; check with the used camera
fov = 53.50  # horizontal fov of camera (degrees)
fov_half = fov / 2.0
image_width = 300
image_height = 300
constant_y = 10  # the constant y in the image corresponding to the LIDAR points
dist_lidar_to_cam_meter = 0.004  # 4 cm between camera lens center to LIDAR laser output
y_image = dist_lidar_to_cam_meter * inches_per_meter * pixels_per_inches  # the constant y in the images

focal_length = 0.0360 * inches_per_meter * pixels_per_inches  # focal length of camera (3.60 mm here)
#coords = []  # the 3D LIDAR coordinates received from Arduino over serial port
positions = []  # the angles associated with each 3D point
distances = []  # the distances associated with each 3D point
#ser = serial.Serial('/dev/tty.usbmodem14301', baudrate=9600)  # the serial port over which to receive LIDAR data

test_with_fake_data = True  # indicates whether to produce fake LIDAR data or not
fake_data = None  # used to producing fake LIDAR data for testing purposes
fake_data_mutex = Lock()  # used for simulating blocking of serial.Serial.readline() call


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
        #time.sleep(0.01)  # simulate delay


def read_fake_data_blocking():
    # blocks until fake LIDAR data has been produced by produce_fake_data_threaded
    global fake_data_mutex, fake_data
    while True:
        fake_data_mutex.acquire()
        if fake_data is not None:
            break
    fake_data_copy = fake_data
    fake_data_mutex.release()
    angle, distance = fake_data_copy.split(" ")
    angle, distance = float(angle), float(distance)
    return angle, distance


def retrieve_object_coords(coords_string):
    # retrieve (parse) the 3D coordinates
    # (might have to convert coordinates from LIDAR
    # from meter units to pixel units through camera's DPI - dots (pixels) per inch)
    x, y, z, distance = coords_string.split(' ')
    return [x, y, z, distance]


def calc_image_point(angle):
    # set 0 angle at center of image since angle is 0-180 left to right
    global y_image
    angle = 90.0 - angle
    x_image =  angle / fov_half * image_width
    return x_image, y_image


def get_lidar_input():
    # returns either fake or real LIDAR data
    if test_with_fake_data:
        return read_fake_data_blocking()
    else:
        input = str(ser.readline())
        input = input[0:len(input) - 2]
        angle, distance = input.split(" ")
        return float(angle), float(distance)


def run(object_detector):
    global focal_length, test_with_fake_data, distances, positions, num_points_per_batch

    # retrieve camera parameters
    #camera_matrix, dist_coefs, rvecs, tvecs = calibrate.calibrate_camera()

    # extract focal length of camera (there is one for x and one for y; take average)
    #focal_length = (camera_matrix[0, 0] + camera_matrix[1, 1]) * 0.5

    # produce fake LIDAR data for testing purposes?
    if test_with_fake_data:
        produce_fake_data_threaded()

    # read LIDAR data 3D coordinates from Arduino (over serial port)
    while True:
        coords_string = " "
        while coords_string != "":
            try:
                # read encoded 3D coordinate over serial port, format: (x, y, z, distance)
                #coords_string = ser.readline()
                #input = str(ser.readline())
                angle, distance = get_lidar_input()
                pos = calc_image_point(float(angle))
                # retrieve the 3D coordinate
                #object_point = retrieve_object_coords(coords_string)
                # store the retrieved 3D coordinate
                #coords.append(object_point[:3])
                positions.append(pos)
                distances.append(distance)
                #distances.append(object_point[3])
            except Exception as e:
                print(e)

            # project 3D points to the camera's 2D plane when enough points are received
            #if len(coords) >= num_points_per_batch:
            if len(positions) >= num_points_per_batch:
                #imagePoints, _ = cv2.projectPoints(coords, rvec, tvec, camera_matrix, dist_coefs)
                #print('\n3D points: {}, \n\n2D points: {}'.format(coords, imagePoints))
                # pass the points to the object-detection script for matching to its detected bounding boxes
                object_detector.receive_projected_points(positions, distances)
                #coords.clear()
                positions.clear()
                distances.clear()
