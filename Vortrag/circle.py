import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

# Draw a red closed circle
cv2.circle(img,(200,200), 40, (0,0,255), -1)
cv2.circle(img,(300,300), 40, (255,0,255), 1)

#Display the image
cv2.imshow("img",img)

cv2.waitKey(0)