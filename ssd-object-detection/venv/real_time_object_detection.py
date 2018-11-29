# USAGE
# python real_time_object_detection.py --prototxt MobileNetSSD_deploy.prototxt.txt --model MobileNetSSD_deploy.caffemodel

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import projection
from threading import Thread, Lock
import numpy as np
import argparse
import imutils
import time
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-p", "--prototxt", required=False, default="MobileNetSSD_deploy.prototxt.txt",
                help="path to Caffe 'deploy' prototxt file")
ap.add_argument("-m", "--model", required=False, default="MobileNetSSD_deploy.caffemodel",
                help="path to Caffe pre-trained model")
ap.add_argument("-c", "--confidence", type=float, default=0.2,
                help="minimum probability to filter weak detections")
args = vars(ap.parse_args())

# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(args["prototxt"], args["model"])

# initialize the video stream and allow the cammera sensor to warmup
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
# initialize the FPS counter
fps = FPS().start()

# used for matching projected LIDAR points with bounding boxes
boxes = []
image_points = []
mutex = Lock()

# initialize and start background thread to receive, project and hand over projected LIDAR points
project_thread = Thread(target=projection.run, args=projection_callback)
project_thread.daemon = True
project_thread.start()


# called from the projection script when a new batch of points have been received  & projected
def projection_callback(projected_object_coords):
    global image_points, boxes, mutex
    # acquire lock for image_points to prevent concurrent updates
    mutex.acquire()
    image_points = projected_object_coords
    mutex.release()


# represents a bounding box, along with projected points falling within its bounds
class BoundingBox:
    def _init__(self, box_coords, idx):
        self.box = box_coords
        self.class_id = idx
        self.projected_points = []
        self.dimensions = [0, 0]
        self.pixel_dimensions = (box[2] - box[0], box[3] - box[1])

    # determines if point is within this box's bounds
    def is_inside(self, point):
        return (point[0] > self.box[0] and point[0] < self.box[2] and point[1] > self.box[1] and point[1] < self.box[3])


# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    # grab the frame dimensions and convert it to a blob
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    net.setInput(blob)
    detections = net.forward()

    # clear the stored bounding boxes from previous frame
    boxes.clear()

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with
        # the prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence > args["confidence"]:
            # extract the index of the class label from the
            # `detections`, then compute the (x, y)-coordinates of
            # the bounding box for the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            boxes.append(BoundingBox([startX, startY, endX, endY], idx))

            # draw the prediction on the frame
            label = "{}: {:.2f}%".format(CLASSES[idx],
                                         confidence * 100)
            cv2.rectangle(frame, (startX, startY), (endX, endY),
                          COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(frame, label, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

    # new batch of projected LIDAR points received?
    if len(image_points) > 0:
        # acquire lock for image_points to prevent concurrent updates
        mutex.acquire()
        # loop over projected points
        for point in image_points:
            # loop over the bounding boxes
            for box in boxes:
                if box.is_inside(point):
                    box.projected_points.append(point)
                    break

        # determine real distance and real dimensions to every bounding box
        for box in boxes:
            # calculate distance
            box.distance = min(box.projected_points)
            # calculate width
            pixel_width = box.box[2] - box.box[0]
            box.dimensions[0] = (pixel_width / projection.focal_length) * box.distance
            # calculate height
            box.dimensions[1] = (box.pixel_dimensions[1] / box.pixel_dimensions[0]) * box.dimensions[0]

        # wait for next batch of points from background thread
        image_points.clear()
        mutex.release()

    # plot dimensions and distance for every bounding box on the current frame
    for box in boxes:
        x = box.box[0]
        y = box.box[3] + 15 if box.box[3] + 15 < frame.shape[1] else box.box[3] - 15
        label = "Width: {:.2f}, height: {:.2f}, distance: {:.2f}".format(box.dimensions[0],
                                                                         box.dimensions[1],
                                                                         box.distance)
        cv2.putText(frame, label, (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[box.class_id], 2)

    # show the output frame
    cv2.imshow("Object distance and dimensions", frame)
    key = cv2.waitKey(1) & 0xFF

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()