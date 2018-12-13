
"""
Camera calibration for distorted images with chess board samples:
reads distorted images and calculates and returns the calibration.
"""

# import the necessary packages
import numpy as np
import cv2 as cv
from common import splitfn
import os
import sys
import getopt
from glob import glob
import json
import codecs

# global variables and constants
debug_dir = './output/'
pattern_size = (9, 6)
pattern_points = []


def process_image(fn):
    # processes one of the calibration images
    global pattern_points
    print('processing %s... ' % fn)
    img = cv.imread(fn, 0)
    if img is None:
        print("Failed to load", fn)
        return None

    found, corners = cv.findChessboardCorners(img, pattern_size)
    if found:
        term = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 30, 0.1)
        cv.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

    if debug_dir:
        vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        cv.drawChessboardCorners(vis, pattern_size, corners, found)
        _path, name, _ext = splitfn(fn)
        outfile = os.path.join(debug_dir, name + '_chess.png')
        cv.imwrite(outfile, vis)

    if not found:
        print('chessboard not found')
        return None

    print('           %s... OK' % fn)
    # return image points and object points for the image
    return corners.reshape(-1, 2), pattern_points


def calibrate_camera(square_size=1.0, num_threads=4, img_mask='./data/left*.jpg'):
    # calibrates camera and returns camera parameters
    global pattern_points
    img_names = glob(img_mask)
    if not os.path.isdir(debug_dir):
        os.mkdir(debug_dir)

    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = cv.imread(img_names[0], cv.IMREAD_GRAYSCALE).shape[:2]

    if num_threads <= 1:
        chessboards = [processImage(fn) for fn in img_names]
    else:
        print("Run with %d threads..." % num_threads)
        from multiprocessing.dummy import Pool as ThreadPool
        pool = ThreadPool(num_threads)
        chessboards = pool.map(process_image, img_names)

    chessboards = [x for x in chessboards if x is not None]
    for (corners, pattern_points) in chessboards:
        img_points.append(corners)
        obj_points.append(pattern_points)

    # calculate the camera parameters
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv.calibrateCamera(obj_points, img_points, (w, h), None, None)

    # return the camera parameters
    return camera_matrix, dist_coefs, np.array(rvecs), np.array(tvecs)
