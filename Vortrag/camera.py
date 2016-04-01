#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2, time
from common import draw_str
cv2.namedWindow("Camera")
capture = cv2.VideoCapture(0)
w = 640
h = 480
capture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, w)
capture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, h)
ret, img = capture.read()
if img == None:
    print "No webcam"
else:
    while 1:
        t0=time.time()
        ret, img = capture.read()
        fps = capture.get(cv2.cv.CV_CAP_PROP_FPS)
        t = int((time.time()-t0)*100)
        draw_str(img, (int(w/32),int(w/12)), "time ms: "+str(t), w/320)
        cv2.imshow("Camera", img)
        k = cv2.waitKey(1)
        if k == 27: # ESC key
            break
cv2.destroyWindow("Camera")
