import cv2
import numpy as np
import PIL

# def create_rec(p0, p1):
	
# 	P0, P1 = p0, p1
# 	sim_photo = np.zeros([1024, 768, 3])

# 	sim_photo[0:20, 0:20, 2] = 255

# 	return sim_photo
# photo_np = create_rec(0, 0)

b = np.random.randint(0, 255, (200, 300), dtype=np.uint8)
g = np.random.randint(0, 255, (200, 300), dtype=np.uint8)
r = np.random.randint(0, 255, (200, 300), dtype=np.uint8)
# b = np.zeros((200, 300), dtype=np.uint8) + 255
# r = np.zeros((200, 300), dtype=np.uint8) + 255
# g = np.zeros((200, 300), dtype=np.uint8) + 255
r[20:60, 80:200] = 255 
b[20:60, 80:200] = 0
g[20:60, 80:200] = 0

b[30:50, 90:190] = 255
g[30:50, 90:190] = 255

img = cv2.merge([b, g, r])

# photo_cv = cv2.fromar(photo_np)
# img = cv2.imread('test.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret,binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# the order of color in cv is [b,g,r]
# draw_img0 = cv2.drawContours(img.copy(),contours,0,(0,255,255),3)
# draw_img1 = cv2.drawContours(img.copy(),contours,1,(255,0,255),3)
# draw_img2 = cv2.drawContours(img.copy(),contours,2,(255,255,0),3)
draw_img3 = cv2.drawContours(img.copy(), contours, -1, (255, 255, 0), 3)


print("contours:",type(contours))
print("numbers",len(contours))

cv2.imshow("img", img)
# cv2.imshow("draw_img0", draw_img0)
# cv2.imshow("draw_img1", draw_img1)
# cv2.imshow("draw_img2", draw_img2)
cv2.imshow("draw_img3", draw_img3)

k = cv2.waitKey(0)
if k == 27:
	cv2.destroyAllWindows()

