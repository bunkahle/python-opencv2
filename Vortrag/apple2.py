#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2

src = cv2.imread("apple.bmp")

threshold = 120
maxValue = 255

colorThresh = src.copy()

# gray = np.zeros((src.shape[0], src.shape[1], 3), np.uint8)

gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
grayThresh = gray.copy()
cv2.namedWindow("src", 1)
cv2.imshow("src", src)
cv2.namedWindow("gray", 1)
cv2.imshow("gray", gray)
ret, colorThresh = cv2.threshold(src, threshold, maxValue, 0)
# cv.Threshold(src, colorThresh, threshold, maxValue, cv.CV_THRESH_BINARY)
ret, grayThresh = cv2.threshold(gray, threshold, maxValue, 0)
# cv.Threshold(gray, grayThresh, threshold, maxValue, cv.CV_THRESH_BINARY)
cv2.namedWindow("colorThresh", 1)
cv2.imshow("colorThresh", colorThresh)
cv2.namedWindow("grayThresh", 1); 
cv2.imshow("grayThresh", grayThresh)
cv2.waitKey(0)
cv2.destroyWindow("src")
cv2.destroyWindow("colorThresh")
cv2.destroyWindow("gray")
cv2.destroyWindow("grayThresh")
