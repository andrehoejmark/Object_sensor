
"""
Just exists for testing serial port communication.
"""

import serial
from threading import Thread
import time

num_points_per_batch = 160  # project and pass along this many points at a time
positions = []  # the angles associated with each LIDAR point
distances = []  # the distances associated with each LIDAR point
#ser = serial.Serial('/dev/ttyACM0', baudrate=9600)
#ser = serial.Serial(port='/dev/tty.usbmodem14301', baudrate=9600)


def run(object_detector):
    # read LIDAR data from Arduino over serial port
    i = 0
    while True:
        try:
            time.sleep(1)
            #pos = " "
            #while pos != "":
            #input = str(ser.readline())
            #input = pos[0:len(pos) - 2]
            #angle, distance = input.split(" ")
            #positions.append(pos)
            i += 1
            # if len(pos) % num_points_per_batch == 0:
            #object_detector.receive_points(positions)
            #positions.clear()
            if i % 4 == 0:
                print("Passing over received points!")
                object_detector.receive_points([1, 2, 3, 4, 5])
        except Exception as e:
            print(e)


