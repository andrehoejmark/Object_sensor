
"""
Just exists for testing serial port communication.
"""

import project
from threading import Thread
import copy
import time


class Detector(object):

    def __init__(self):
        self.positions = []

    def receive_points(self, positions):
        self.positions = copy.deepcopy(positions)

    def run(self):
        while True:
            try:
                time.sleep(1)
                if len(self.positions) > 0:
                    print(self.positions)
                    self.positions.clear()
            except Exception as e:
                print(e)


if __name__ == '__main__':
    object_detector = Detector()

    project_thread = Thread(target=project.run, args=(object_detector, ))
    project_thread.daemon = True
    project_thread.start()

    object_detector.run()
