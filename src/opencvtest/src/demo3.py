import numpy as np
import cv2
import time

time_start = time.time()
img = cv2.imread('123.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
gray = cv2.cvtColor(hsv, cv2.COLOR_BGR2GRAY)
ret, binary = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
draw_img3 = cv2.drawContours(img.copy(), contours, -1, (255, 255, 0), 3)

# (B, G, R) = cv2.split(img)
# retR, binaryR = cv2.threshold(R, 130, 255, cv2.THRESH_BINARY)
# contours, hierarchy = cv2.findContours(binaryR, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# draw_img3 = cv2.drawContours(R.copy(), contours, 1, (255, 255, 0), 3)

cv2.imshow("img", gray)
cv2.imshow("binary", binary)
# cv2.imshow("B", B)
# cv2.imshow("G", G)

# cv2.imshow("R", R)

a, b, _ = np.where(draw_img3 == 0)

a0 = int(np.mean(a))
b0 = int(np.mean(b))

print(a0)
print(b0)

print(time.time() - time_start)
# cv2.circle(draw_img3,(459, 719),10,(0,255,155),3)
cv2.imshow("draw", draw_img3)

k = cv2.waitKey(0)
if k == 27:
	cv2.destroyAllWindows()
