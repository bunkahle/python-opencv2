import pylab
import cv2

a = pylab.imread("lena.jpg")
b = cv2.cvtColor(a,cv2.COLOR_RGB2BGR)
cv2.imshow("Test", b)
cv2.waitKey(0)