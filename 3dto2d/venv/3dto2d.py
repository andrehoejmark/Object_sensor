
'''
Reads LIDAR point cloud data [3D-coordinates) from some memory location,
projects these to the 2D camera-plane and writes the results to some memory location.
'''

# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2 as cv

# built-in modules
import os
import sys

if __name__ == '__main__':
    # load camera calibration data:
    with open('data/calib.json') as jsonFile:
        jsonData = json.load(jsonFile)
    if not jsonData:
        print("Failed opening camera calibration file - make sure you run calibrate.py first!")
        sys.exit()

    # retrieve camera parameters:
    camera_matrix = jsonData['camera_matrix']
    dist_coefs = jsonData['dist_coefs']
    rvecs = jsonData['rvecs']
    tvecs = jsonData['tvecs']

    # load LIDAR point cloud data (3D-coordinates)
    with open('data/lidarData.json') as jsonFile:
        jsonData = json.load(jsonFile)
    if not jsonData:
        print("Failed opening LIDAR point cloud data file - make sure LIDAR program is running and saving data!")
        sys.exit()

    # retrieve the objectData (3D-coordinates)
    objectPoints = jsonData['lidarCoordinates']

    # project 3D points to camera 2D plane with:
    # cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs[, imagePoints[, jacobian[, aspectRatio]]]) → imagePoints, jacobian
    imagePoints, _ = cv2.projectPoints(objectPoints, rvec, tvec, camera_matrix, dist_coefs)

    # write projected coordinates to file for processing somewhere else
    jsonData = { 'imagePoints': imagePoints }
    with open('data/projectedLidarData.json', 'w') as jsonFile:
        json.dump(jsonData, jsonFile)