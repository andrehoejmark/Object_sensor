import os
import cv2
import numpy as np
import tensorflow as tf
import argparse
import sys
import label_map_util
import visualization_utils as vis_util
from threading import Thread, Lock
import projection
import time
import copy

# Set up camera constants
IM_WIDTH = 1280
IM_HEIGHT = 720
# Use smaller resolution for slightly faster framerate
#IM_WIDTH = 640
#IM_HEIGHT = 480

# Select camera type (if user enters --usbcam when calling this script,
# a USB webcam will be used)
camera_type = 'picamera'
parser = argparse.ArgumentParser()
parser.add_argument('--usbcam', help='Use a USB webcam instead of picamera',
                    action='store_true')
args = parser.parse_args()
if args.usbcam:
    camera_type = 'usb'

# This is needed since the working directory is the object_detection folder.
sys.path.append('..')

# Name of the directory containing the object detection module we're using
MODEL_NAME = 'ssdlite_mobilenet_v2_coco_2018_05_09'

# Grab path to current working directory
CWD_PATH = os.getcwd()

# Path to frozen detection graph .pb file, which contains the model that is used
# for object detection.
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, 'frozen_inference_graph.pb')

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH, 'data', 'mscoco_label_map.pbtxt')

# Number of classes the object detector can identify
NUM_CLASSES = 90

## Load the label map.
# Label maps map indices to category names, so that when the convolution
# network predicts `5`, we know that this corresponds to `airplane`.
# Here we use internal utility functions, but anything that returns a
# dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Load the Tensorflow model into memory.
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString(serialized_graph)
        tf.import_graph_def(od_graph_def, name='')

    sess = tf.Session(graph=detection_graph)


# Define input and output tensors (i.e. data) for the object detection classifier

# Input tensor is the image
image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

# Output tensors are the detection boxes, scores, and classes
# Each box represents a part of the image where a particular object was detected
detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

# Each score represents level of confidence for each of the objects.
# The score is shown on the result image, together with the class label.
detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

# Number of objects detected
num_detections = detection_graph.get_tensor_by_name('num_detections:0')

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()
font = cv2.FONT_HERSHEY_SIMPLEX

# Used for matching projected points with bounding boxes
bounding_boxes = []
image_points = []
distances = []
angles = []
mutex = Lock()

# Initialize camera and perform object detection.
# The camera has to be set up and used differently depending on if it's a
# Picamera or USB webcam.


# Represents a bounding box, along with projected points falling within its bounds
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
        return self.box[0] <= point[0] <= self.box[2] and self.box[1] <= point[1] <= self.box[3]


class SSDObjectDetector:

    # Called from the projection script when a new batch of points is ready
    def receive_points(self, projected_positions, object_distances):
        global image_points, distances, angles, mutex

        # acquire lock for image_points to prevent concurrent updates
        mutex.acquire()
        # make deep copies of the received data to store; since the originals will be removed after function call
        image_points = copy.deepcopy(projected_positions)
        distances = copy.deepcopy(object_distances)
        mutex.release()

    # Performs the object detection, calculates and plots distances and dimensions
    def perform_detection(self, frame):
        global frame_rate_calc, bounding_boxes, image_points, distances, mutex

        t1 = cv2.getTickCount()

        frame_expanded = np.expand_dims(frame, axis=0)

        # clear the stored bounding boxes from previous frames if new points have been received
        store_new_boxes = False
        mutex.acquire()
        if len(image_points) > 0:
            bounding_boxes.clear()
            store_new_boxes = True
        mutex.release()

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = sess.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: frame_expanded})

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        # Draw the results of the detection (aka 'visualize the results')
        vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            boxes,
            classes,
            scores,
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.40)

        # Loop over all detected bounding boxes and save them to bounding_boxes list
        for i, box in enumerate(boxes):
            # store new bounding box for detected object
            if store_new_boxes:
                box = box * np.array([IM_WIDTH, IM_HEIGHT, IM_WIDTH, IM_HEIGHT])
                (start_x, start_y, end_x, end_y) = box.astype("int")
                bounding_boxes.append(BBox([start_x, start_y, end_x, end_y], classes[i]))

        # acquire lock for image_points
        mutex.acquire()
        # new batch of projected LIDAR points received?
        if len(image_points) > 0:
            # loop over projected points
            for point_index, point in enumerate(image_points):
                # loop over the bounding boxes
                for box in bounding_boxes:
                    if box.is_inside(point):
                        box.projected_points.append(point)
                        box.distances.append(distances[point_index])
                        break

            # determine real distance and real dimensions to every bounding box
            for box in boxes:
                # continue if no projected points fell within this box's bounds
                if len(box.projected_points) == 0:
                    continue
                # estimate distance as the smallest distance of all the projected points within the bounding box
                box.distance = min(box.distances)
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
        for box in bounding_boxes:
            # continue if no projected points fell within this box's bounds
            if len(box.projected_points) == 0:
                continue
            x = box.box[0] - 20
            y = box.box[1] + 20
            # print("Distances in box:" + str(box.distances))
            # print("Points in box:" + str(box.projected_points))
            label = "W:{:.2f}, H:{:.2f}, D:{:.2f}".format(box.dimensions[0],
                                                          box.dimensions[1],
                                                          box.distance)
            cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        COLORS[box.class_id % len(COLORS)], 2)

            # plot the projected points on the bounding box for debugging purposes
            for index, point in enumerate(box.projected_points):
                cv2.circle(frame, (int(point[0]), int(point[1])), 2, (0, 255, 255), 1)

        cv2.putText(frame, "FPS: {0:.2f}".format(frame_rate_calc), (30, 50), font, 1,
                    (255, 255, 0), 2, cv2.LINE_AA)

        # All the results have been drawn on the frame, so it's time to display it.
        cv2.imshow('Object Distance and Dimensions Calculator', frame)

        t2 = cv2.getTickCount()
        time1 = (t2 - t1) / freq
        frame_rate_calc = 1.0 / time1

        # Press 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            return False
        return True

    # Runs the object detector
    def run(self):
        ### Picamera ###
        if camera_type == 'picamera':
            # import picamera modules
            from picamera.array import PiRGBArray
            from picamera import PiCamera

            # Initialize Picamera and grab reference to the raw capture
            camera = PiCamera()
            camera.resolution = (IM_WIDTH, IM_HEIGHT)
            camera.framerate = 10
            raw_capture = PiRGBArray(camera, size=(IM_WIDTH, IM_HEIGHT))
            raw_capture.truncate(0)

            for frame1 in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                # Acquire frame from Pi camera threaded video stream
                frame = frame1.array
                frame.setflags(write=1)

                # Run the detection, calculate and plot distances and dimensions
                should_continue = self.run_detection(frame)

                # Did user press 'q'?
                if not should_continue:
                    break

                raw_capture.truncate(0)

            camera.close()

        ### USB webcam ###
        elif camera_type == 'usb':
            # Initialize USB webcam feed
            camera = cv2.VideoCapture(0)
            camera.set(3, IM_WIDTH)
            camera.set(4, IM_HEIGHT)

            while True:
                # Acquire frame from threaded video stream
                ret, frame = camera.read()

                # Run the detection, calculate and plot distances and dimensions
                should_continue = self.run_detection(frame)

                # Did user press 'q'?
                if not should_continue:
                    break

            camera.release()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    # create object detector instance
    object_detector = SSDObjectDetector()

    # initialize and start background thread to receive, project and hand over projected LIDAR points
    project_thread = Thread(target=projection.run, args=(object_detector, ))
    project_thread.daemon = True
    project_thread.start()

    # run object detector
    object_detector.run()
