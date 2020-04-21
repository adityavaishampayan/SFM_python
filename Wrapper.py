import cv2
import numpy as np
#froms scripts.UndistortImage import *

img1 = cv2.imread(r"Dataset/Data/1.jpg")
cv2.imshow("image1",img1)

img2 = cv2.imread(r"Dataset/Data/2.jpg")
cv2.imshow("image2",img2)

# K = [568.996140852     0            643.21055941;
#           0       568.988362396     477.982801038;
#           0            0                       1]

# undistorted_image1 = UndistortImage(img1, K)
# undistorted_image2 = UndistortImage(img2, K)


# Initiate SIFT detector
sift = cv2.xfeatures2d.SIFT_create()

# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(undistorted_image1,None)
kp2, des2 = sift.detectAndCompute(undistorted_image2,None)

# BFMatcher with default params
bf = cv2.BFMatcher()
matches = bf.knnMatch(des1,des2, k=2)

ret = []
idx1, idx2 = [], []
for m,n in matches:
    if m.distance < 0.75 * n.distance:
        p1 = frame_1.kps[m.queryIdx]
        p2 = frame_2.kps[m.trainIdx]
        idx1.append(m.queryIdx)
        idx2.append(m.trainIdx)
        ret.append((p1, p2))

ret = np.array(ret)
idx1 = np.array(idx1, dtype=np.int32)
idx2 = np.array(idx2, dtype=np.int32)

E, mask = cv2.findEssentialMat(ret[:,0], ret[:,1], frame_1.K, cv2.RANSAC)
mask = np.array(mask, dtype = np.bool)
print(idx1[~mask.ravel()])
print(idx2[~mask.ravel()])
print(E)

cv2.drawMatchesKnn expects list of lists as matches.
outImg = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,flags=2)
cv2.imshow("img3",outImg)

cv2.waitKey(0)
cv2.destroyAllWindows()
