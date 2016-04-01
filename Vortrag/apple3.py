#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2

src = cv2.imread("apple.bmp")
gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
cannyImg = cv2.Canny(gray, 75, 250)
cv2.imshow("src", src)
cv2.imshow("canny", cannyImg)
cv2.waitKey(0)
#cv2.destroyWindow("src")
#cv2.destroyWindow("canny")

