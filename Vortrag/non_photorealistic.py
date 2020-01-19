import cv2
import numpy as np
img = cv2.imread("test3.jpg")
cv2.imshow("test", img)
# sketch_gray, sketch_color = cv2.pencilSketch(img, sigma_s=60, sigma_r=0.07, shade_factor=0.0)

def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

image = increase_brightness(img, value=0)
cv2.imshow("bright", image)
output = cv2.blur(image,(5,5))
output = cv2.stylization(output, sigma_s=60, sigma_r=0.05)

cv2.imshow("output", output)

cv2.waitKey()
