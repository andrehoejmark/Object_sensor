
"""
Detects objects from video in real-time, and estimates and displays distance to
and dimensions of these objects.
"""

# import the necessary packages
from imutils.video import VideoStream
import projection
from threading import Thread, Lock
import numpy as np
import cv2
import pandas as pd
import argparse
import imutils
import time
import copy

# object detector parameters
prototxt = "MobileNetSSD_deploy.prototxt.txt"
model = "MobileNetSSD_deploy.caffemodel"
min_confidence = 0.2

# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
net = cv2.dnn.readNetFromCaffe(prototxt, model)

# used for matching projected LIDAR points with bounding boxes
boxes = []
image_points = []
distances = []
angles = []
mutex = Lock()

# used for fps calculation
tick_freq = cv2.getTickFrequency()

# indicates whether to test with real LIDAR test data or not
test_with_test_data = True


# represents a bounding box, along with projected points falling within its bounds
class BBox:

    def __init__(self, box_coords, idx):
        self.box = box_coords
        self.class_id = idx
        self.projected_points = []
        self.distances = []
        self.angles = []
        self.dimensions = [0, 0]
        self.pixel_dimensions = (self.box[2] - self.box[0], self.box[3] - self.box[1])

    # determines if point is within this box's bounds
    def is_inside(self, point):
        return (point[0] > self.box[0] and point[0] < self.box[2] and point[1] > self.box[1] and point[1] < self.box[3])


class SSDObjectDetector:

    # called from the projection script when a new batch of points has been received & projected
    def receive_projected_points(self, projected_object_coords, object_distances, object_angles):
        global image_points, distances, angles, mutex
        # acquire lock for image_points to prevent concurrent updates
        mutex.acquire()
        # make deep copies of the received data to store; since the originals will be removed after function call
        image_points = copy.deepcopy(projected_object_coords)
        distances = copy.deepcopy(object_distances)
        angles = copy.deepcopy(object_angles)
        mutex.release()

    def run(self):
        global boxes, image_points, distances, mutex
        # initialize the video stream and allow the cammera sensor to warmup
        vs = VideoStream(src=0).start()
        time.sleep(2.0)
        # loop over the frames from the video stream
        while True:
            # used for fps calculation
            start_time = cv2.getTickCount()
            if test_with_test_data:  # test with test image
                # grab the test image from disk
                frame = cv2.imread("data/framedata.png", 1)
            else:  # use camera to catch frames
                # grab the frame from the threaded video stream
                frame = vs.read()

            # resize frame to have a maximum width of 400 pixels
            imutils.resize(frame, 400)

            # grab the frame dimensions and convert it to a blob
            (h, w) = frame.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

            # pass the blob through the network and obtain the detections and
            # predictions
            net.setInput(blob)
            detections = net.forward()

            # clear the stored bounding boxes from previous frames if new points have been received
            if len(image_points) > 0:
                boxes.clear()

            # loop over the detections
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e. probability) associated with
                # the prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the confidence is
                # greater than the minimum confidence
                if confidence > min_confidence:
                    # extract the index of the class label from the
                    # detections, then compute the (x, y)-coordinates of
                    # the bounding box for the object
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # store new bounding box for detected object
                    if len(image_points) > 0:
                        boxes.append(BBox([startX, startY, endX, endY], idx))

                    # draw the prediction on the frame
                    label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[idx], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            # new batch of projected LIDAR points received?
            if len(image_points) > 0:
                # acquire lock for image_points to prevent concurrent updates
                mutex.acquire()
                # loop over projected points
                for point_index, point in enumerate(image_points):
                    # loop over the bounding boxes
                    for box in boxes:
                        if box.is_inside(point):
                            box.projected_points.append(point)
                            box.distances.append(distances[point_index])
                            box.angles.append(angles[point_index])
                            break

                # determine real distance and real dimensions to every bounding box
                for box in boxes:
                    # continue if no projected points fell within this box's bounds
                    if len(box.projected_points) == 0:
                        continue
                    # estimate distance as the smallest distance of all the projected points within the bounding box
                    box.distance = min(box.distances)
                    box.angle = min(box.angles)
                    # estimate real width of detected object
                    pixel_width = box.box[2] - box.box[0]
                    box.dimensions[0] = (pixel_width / projection.focal_length) * box.distance
                    # estimate real height of detected object
                    box.dimensions[1] = (box.pixel_dimensions[1] / box.pixel_dimensions[0]) * box.dimensions[0]

                # wait for next batch of points from background thread (received in a callback function)
                image_points.clear()
                distances.clear()
                mutex.release()

            # plot dimensions of and distance to every detected object on the current frame
            for box in boxes:
                # continue if no projected points fell within this box's bounds
                if len(box.projected_points) == 0:
                    continue
                x = box.box[0] - 20
                y = box.box[1] + 20
                print("Distances in box:" + str(box.distances))
                print("Points in box:" + str(box.projected_points))
                print("Angles in box:" + str(box.angles))
                label = "W:{:.2f},H:{:.2f},D:{:.2f}".format(box.dimensions[0],
                                                            box.dimensions[1],
                                                            box.distance)
                cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[box.class_id], 2)

                # plot the projected points on the bounding box for debugging purposes
                if test_with_test_data:
                    for index, point in enumerate(box.projected_points):
                        #print("Point: " + str(point))
                        #print("Distance: " + str(box.distances[index]))
                        #print("Min distance" + str(box.distance))
                        #print("Min angle" + str(box.angle))
                        cv2.circle(frame, (int(point[0]), int(point[1])), 2, (0, 255, 255), 1)


            # show the output frame
            cv2.imshow("Object Distance and Dimensions", frame)
            key = cv2.waitKey(1) & 0xFF

            # calculate and display the fps of the current frame
            end_time = cv2.getTickCount()
            fps = (end_time - start_time) / tick_freq
            label = "{:.2f} FPS".format(fps)
            cv2.putText(frame, label, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # break from the loop if 'q' was pressed
            if key == ord("q"):
                break

        # cleanup
        cv2.destroyAllWindows()
        vs.stop()


if __name__ == '__main__':
    # create object detector instance
    object_detector = SSDObjectDetector()

    # initialize and start background thread to receive, project and hand over projected LIDAR points
    project_thread = Thread(target=projection.run, args=(object_detector, ))
    project_thread.daemon = True
    project_thread.start()

    # run object detector
    object_detector.run()
