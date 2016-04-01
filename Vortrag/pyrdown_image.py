import cv2
import sys

def doPyrDown(im):
    # Do the pyrdown by default half size
    out = cv2.pyrDown(im)
    return out
    
if len(sys.argv) == 1:
    filename = "sourceImage1.jpg"
else:
    filename = sys.argv[1]
img = cv2.imread(filename)
# Create a window to show our input image
cv2.imshow("Original-in", img)
# Create an image to hold the smoothed output
#    
ret = doPyrDown(img)
cv2.imshow("Pyrdown-out", ret)
cv2.waitKey(0)
cv2.destroyWindow("Original-in")
cv2.destroyWindow("Pyrdown-out")
