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
import math


ser = serial.Serial('/dev/ttyACM0', baudrate=9600) # the serial port
degrees = []  # the angles associated with each point
distances = []  # the distances associated with each point

def get_lidar_input():
    global ser, degrees, distances
    try:
        degrees, distance = ser.readline().decode('utf8', 'ignore').rstrip().split(",")
    except ValueError as err:
        print(err)
        return 0.0, 0.0
    return float(degrees), float(distance)



def mean_distance(array):
    total = 0.0
    for i in range(len(array)-1):
        total += math.fabs(array[i]-array[i+1])

    return 1.5 * (total / (len(array)-1))


def find_index_object(mean_dist, array_dist):
    start_end_index = []

    prev_dist = array_dist[0]
    for i in range(len(array_dist) - 1):
        dist_between_points = math.fabs(prev_dist - array_dist[i + 1])

        if (dist_between_points > mean_dist):
            if (len(start_end_index) == 0):
                start_end_index.append(i + 1)
            elif (len(start_end_index) == 1):
                start_end_index.append(i)

        if (len(start_end_index) == 2):
            return start_end_index[0], start_end_index[1]

        prev_dist = array_dist[i + 1]

    return None, None


def find_width(mean_dist, array_dist, array_deg):
    start_index, end_index = find_index_object(mean_dist, array_dist)

    if (start_index != None):
        # Cosinussatsen
        A = math.fabs(array_deg[start_index] - array_deg[end_index])
        b = array_dist[start_index]
        c = array_dist[end_index]

        width = math.sqrt(b ** 2 + c ** 2 - 2 * b * c * math.cos(math.radian(A)))
        return width
    return None




def run(object_detector):
    global degrees, distances
    while True:
        try:
            degree, distance = get_lidar_input()

            # Sent message to know when the lidar has changed rotation.
            if(degree == "None" and distance == "None"):

                print(find_width(mean_distance(distances), distances, degrees))
                degrees.clear()
                distances.clear()
            else:
                distances.append(distance)
                degrees.append(degree)

        except Exception as E:
            print(E)
