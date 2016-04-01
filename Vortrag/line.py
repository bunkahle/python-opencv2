import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

# Draw a blue line with thickness of 5 px
cv2.line(img,(15,20),(70,50),(255,0,0),5)

#Display the image
cv2.imshow("img",img)

cv2.waitKey(0)