#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2

src = cv2.imread("apple.bmp")
cv2.imshow("orig", src)
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray,127,255,0)

# find the contours
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# draw the contour
countour_img = src.copy()
countour_img[:] = 0
cv2.drawContours(src, contours, -1, (0,255,0), 3)
cv2.drawContours(countour_img, contours, -1, (0,255,0), 3)
cv2.imshow("src", src)
cv2.imshow("contours", countour_img)
cv2.waitKey(0)

