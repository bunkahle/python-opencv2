#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
optional input parameters:

"""

import sys
import cv2
import time

input_name = "lena.jpg"
input_name = "lena_full.jpg"
input_name = "lena_old_full.jpg"
input_name = "lena_old_full.jpg"
input_name = "lena_young_old_full.jpg"
input_name = "faces.jpg"
input_name = "faces2.jpg"
input_name = "family.jpg" # image_scale = 3 and image_scale = 1.5
input_name = "hillary.jpg" # image_scale = 1 or image_scale = 2 or image_scale = 3
input_name = "hillary2.jpg"
image_scale = 1

# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned 
# for accurate yet slow object detection. For a faster operation on real video 
# images the settings are: 
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING, 
# min_size=<minimum possible face size

min_size = (30, 30)

# good values for best face detection
haar_scale = 1.7
min_neighbors = 1
# for fast detection within a camera capture better take:
#haar_scale = 1.5
#min_neighbors = 2
haar_flags = cv2.cv.CV_HAAR_SCALE_IMAGE
cascade_fn = "haarcascades/haarcascade_frontalface_alt.xml"
cascade = cv2.CascadeClassifier(cascade_fn)

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

if __name__ == '__main__':
    print sys.argv
    if len(sys.argv) > 1:
        input_name = sys.argv[1]
    if input_name.isdigit():
        capture = cv2.VideoCapture(input_name)
        w = 640
        h = 480
        capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, w)
        capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, h)
    else:
        capture = None
    cv2.namedWindow("result", 1)
    if capture:
        frame_copy = None
        while True:
            frame = cv.QueryFrame(capture)
            if not frame:
                cv.WaitKey(0)
                break
            if not frame_copy:
                frame_copy = cv.CreateImage((frame.width,frame.height),
                                            cv.IPL_DEPTH_8U, frame.nChannels)
            if frame.origin == cv.IPL_ORIGIN_TL:
                cv.Copy(frame, frame_copy)
            else:
                cv.Flip(frame, frame_copy, 0)
            detect_and_draw(frame_copy, cascade)
            if cv.WaitKey(10) >= 0:
                break
    else:
        image = cv2.imread(input_name, 1)
        a = time.time()
        rects = detect(image, cascade)
        vis = image.copy()
        draw_rects(vis, rects, (0, 0, 255))
        print "Elapsed time:", time.time()-a
        cv2.imshow("facedetect", vis)
        cv2.waitKey(0)
    cv2.destroyWindow("result")
