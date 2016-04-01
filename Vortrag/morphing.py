#!/usr/bin/python
# -*- coding: utf-8 -*-
#

import numpy as np
import cv2

def CreateImage((width, height), bits=np.uint8, channels=3, color=(0, 0, 0)): # (cv.GetSize (frame), 8, 3)
    """Create new image(numpy array) filled with certain color in RGB"""
    # Create black blank image
    image = np.zeros((height, width, channels), bits)
    # Fill image with color
    image[:] = color
    return image

# sourceImage1 = cv.LoadImage("obama.jpg")
# sourceImage2 = cv.LoadImage("osama.jpg")
sourceImage1 = cv2.imread("obama2.jpg")
sourceImage2 = cv2.imread("bush.jpg")
sourceImage1FaceWithMorphology = CreateImage(tuple(sourceImage1.shape[:2]))
sourceImage2FaceWithMorphology = CreateImage(tuple(sourceImage2.shape[:2]))
morphedImage = CreateImage(tuple(sourceImage1.shape[:2]))
sourceImage1FaceWithMorphology = sourceImage1.copy()
sourceImage2FaceWithMorphology = sourceImage2.copy()
# set Labels
sourceImage1FaceMorphologyWindow = "Source Image 1"
sourceImage2FaceMorphologyWindow = "Source Image 2"
morphedImageWindow = "Morphed Image"
# create and pos)ition windows
cv2.namedWindow(sourceImage1FaceMorphologyWindow)
cv2.namedWindow(sourceImage2FaceMorphologyWindow)
cv2.namedWindow(morphedImageWindow)
cv2.moveWindow(sourceImage1FaceMorphologyWindow, 0, 0)
cv2.moveWindow(sourceImage2FaceMorphologyWindow, 330, 0)
cv2.moveWindow(morphedImageWindow, 660, 0)

cv2.imshow(sourceImage1FaceMorphologyWindow, sourceImage1FaceWithMorphology)
cv2.imshow(sourceImage2FaceMorphologyWindow, sourceImage2FaceWithMorphology)

c = ""
stop = False
while c != "q" and c !="\x1b":
    for count in range(100):
        morphedImage = cv2.addWeighted(sourceImage1FaceWithMorphology, 1-(count*0.01), sourceImage2FaceWithMorphology, count*0.01, 0)
        cv2.imshow(morphedImageWindow, morphedImage)
        c = cv2.waitKey(21)
        if c != -1:
            if c == ord("q") or c == 27:
                stop = True
                break
            elif c == " ":
                c = cv2.waitKey()
    if stop: break
    for count in range(100):
        cv2.addWeighted(sourceImage2FaceWithMorphology, 1-(count*0.01), sourceImage1FaceWithMorphology, count*0.01, 0, morphedImage)
        cv2.imshow(morphedImageWindow, morphedImage)
        c = cv2.waitKey(21)
        if c != -1:
            if c == ord("q") or c == 27:
                stop = True
                break
            elif c == " ":
                c = cv2.waitKey()
    if stop: break
    
cv2.destroyAllWindows()
