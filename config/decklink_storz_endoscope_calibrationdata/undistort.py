import cv2
import yaml
import numpy as np

raw_img = cv2.imread("left-0000.png")

with open("ost.yaml") as file:
    param = yaml.load(file, yaml.FullLoader)

mtx = param["camera_matrix"]["data"]
dst = param["distortion_coefficients"]["data"]

mtx = np.array(mtx).reshape(3, 3)
dst = np.array(dst)

h,  w = raw_img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dst, (w,h), 1, (w,h))

dst_img = cv2.undistort(raw_img, mtx, dst, None, newcameramtx)

cv2.imshow("raw_img", raw_img)
cv2.imshow("dst_img", dst_img)
cv2.waitKey()
