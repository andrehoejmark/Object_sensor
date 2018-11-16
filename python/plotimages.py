import numpy as np
import cv2
from matplotlib import pyplot as plt
import imutils

# Load video with cv2
cap = cv2.VideoCapture('res/TUD-Crossing.mp4')

# Initial testing on the first frame
for _ in range(1):

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Image operations
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    edges = cv2.Canny(frame, 100, 200)

    edged = cv2.Canny(gray, 50, 100)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    # Contours logic - Not in use here
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]

    # plot meta
    titles = ['Original frame', 'grayscale / GaussianBlur', 'Canny Edges Original', 'Canny Edges blurred']
    images = [frame, gray, edges, edged]

    # plot images
    for i in range(4):
        plt.subplot(2, 2, i + 1), plt.imshow(images[i], 'gray')
        plt.title(titles[i])
        plt.xticks([]), plt.yticks([])
    plt.show()

    # Display the resulting frame
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
