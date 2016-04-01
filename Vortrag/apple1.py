#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def CreateImage((width, height), bits=np.uint8, channels=3, color=(0, 0, 0)): # (cv.GetSize (frame), 8, 3)
    """Create new image(numpy array) filled with certain color in RGB"""
    # Create black blank image
    image = np.zeros((height, width, channels), bits)
    # Fill image with color
    image[:] = color
    return image

image = CreateImage((400,400), color=(0,0,255))
cv2.namedWindow("Image", 1)
cv2.imshow("Image", image)
c = cv2.waitKey(0)
image2 = cv2.imread("apple.bmp")
cv2.namedWindow("Image2", 1)
cv2.imshow("Image2", image2)
print image2.shape
c = cv2.waitKey(0)

# cv.ReleaseImage(image)
cv2.destroyWindow("Image")
