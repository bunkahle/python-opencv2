#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np 
import cv2, sys

def composite(img1, img2, mask0):
    if mask0.shape[2] == 3:
        mask2 = cv2.cvtColor(mask0, cv2.COLOR_BGR2GRAY)
    else:
        mask2 = mask0[:]
    mask1 = np.ones((img1.shape[0], img1.shape[1], 3), np.uint8)
    mask1[..., 0] = mask2
    mask1[..., 1] = mask2
    mask1[..., 2] = mask2
    white = np.ones((img1.shape[0], img1.shape[1], 3), np.uint8)
    white[:] = (0, 0, 0)
    invmask = np.zeros((img1.shape[0], img1.shape[1], 3), np.uint8)
    invmask = cv2.absdiff(white, mask1)
    invmask = cv2.bitwise_not(invmask)
    output = np.zeros((img1.shape[0], img1.shape[1], 3), np.uint8)
    cv2.subtract(img2, invmask, dst=output)
    return output

def find_contours(img):
    img1 = cv2.bitwise_not(img)
    imgray = cv2.cvtColor(img1 ,cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray,127,255,0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_cont = img1.copy()
    cv2.drawContours(img_cont, contours, -1, (255,255,255), -1)
    return img_cont

img1 = cv2.imread('symbol036.jpg')
cv2.imshow("Img1", img1)
lena = cv2.imread("lena_part.jpg")
cv2.imshow("Img2", lena)
cv2.waitKey(0)
eroding = True
if eroding:
    img_erode = cv2.erode(img1, None, iterations=3)
else:
    img_erode = img1[:]
blending_mask = find_contours(img_erode)
cv2.imshow("Mask", blending_mask)
cv2.waitKey(0)
subtract_mask = cv2.subtract(lena, blending_mask)
cv2.imshow("subtract_mask", subtract_mask)
# output2 = composite(img1, img1, blending_mask)
output2 = cv2.bitwise_and(img1, blending_mask)
cv2.imshow("Output2", output2)
output = cv2.add(subtract_mask, output2)
cv2.imshow("Result", output)
cv2.waitKey(0)


