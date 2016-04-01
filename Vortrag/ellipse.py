import numpy as np
import cv2

# Create a black image
img = np.zeros((512,512,3), np.uint8)

# Draw ellipse
cv2.ellipse(img,(200,200),(80,50),0,0,360,(0,255,0),-1)
cv2.ellipse(img,(200,200),(80,50),45,0,360,(0,0,255),1)
cv2.ellipse(img,(200,200),(80,80),0,0,180,(255,0,0),3)

#Display the image
cv2.imshow("img",img)

cv2.waitKey(0)