# -*- coding: utf-8 -*-

"""
MIT License

Copyright (c) 2018 Aditya Vaishampayan, Amrish Bhaskaran

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# @file    Wrapper.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  Full pipeline of structure from motion


import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except :
    pass

import cv2
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import glob

# from scripts.Frame import set_key_pts, set_descriptors, set_pose, set_3d_pts, get_detections, get_pose, get_3d_pts

from scripts.EstimateFundamentalMatrix import get_fundamental_matrix
from scripts.GetInliersRANSAC import get_inliers_ransac
from scripts.EssentialMatrixFromFundamentalMatrix import get_essential_matrix
from scripts.ExtractCameraPose import extract_camera_pose
from scripts.DisambiguateCameraPose import disambiguate_camera_pose
from scripts.NonlinearTriangulation import non_linear_triangulation
from scripts.LinearPnP import linear_pnp
from scripts.NonlinearPnP import non_linear_pnp
from scripts.BuildVisibilityMatrix import build_visibility_matrix
from scripts.BundleAdjustment import bundle_adjustment

K = np.array([[568.996140852, 0, 643.21055941],
              [0, 568.988362396, 477.982801038],
              [0, 0, 1]])

W = np.array([[0, -1, 0],
              [1, 0, 0],
              [0, 0, 1]])

path = r'/home/aditya/SFM_python/Dataset/Data/'

imageRaw1 = cv2.imread(path + '1.jpg')
imageRaw2 = cv2.imread(path + '2.jpg')

#cv2.imshow("image1", imageRaw1)
#cv2.imshow("image2", imageRaw2)

gray1 = cv2.cvtColor(imageRaw1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(imageRaw2, cv2.COLOR_BGR2GRAY)

eq_image1 = cv2.equalizeHist(gray1)
eq_image2 = cv2.equalizeHist(gray2)

#cv2.imshow("equalised image1", eq_image1)
#cv2.imshow("equalised image2", eq_image2)

orb = cv2.ORB_create()
kp1, des1 = orb.detectAndCompute(eq_image1, None)
kp2, des2 = orb.detectAndCompute(eq_image2, None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# Match descriptors.
matches = bf.match(des1, des2)
# Sort them in the order of their distance.
matches = sorted(matches, key=lambda x: x.distance)

draw_params = dict(matchColor = (0,255,0),
                   singlePointColor=(255, 0, 0),
                   flags = 0)
#
# index_params = dict(algorithm=6,
#                     table_number=6,
#                     key_size=12,
#                     multi_probe_level=2)
# search_params = {}
# flann = cv2.FlannBasedMatcher(index_params, search_params)
# matches = flann.knnMatch(des1, des2, k=2)

# As per Lowe's ratio test to filter good matches
good_points = []
for match in matches:
    good_points.append(match)

# Draw first 10 matches.
img3 = cv2.drawMatches(gray1, kp1, gray2, kp2, good_points, None, **draw_params)
plt.imshow(img3), plt.show()

pts_l = np.float32([kp1[match.queryIdx].pt for match in good_points])
pts_r = np.float32([kp2[match.trainIdx].pt for match in good_points])

pts_l_norm = cv2.undistortPoints(np.expand_dims(pts_l, axis=1), cameraMatrix=K, distCoeffs=None)
pts_r_norm = cv2.undistortPoints(np.expand_dims(pts_r, axis=1), cameraMatrix=K, distCoeffs=None)

E, mask = cv2.findEssentialMat(pts_l_norm, pts_r_norm, focal=1.0, pp=(0., 0.), method=cv2.RANSAC, prob=0.999, threshold=3.0)
points, R, t, mask = cv2.recoverPose(E, pts_l_norm, pts_r_norm)

M_r = np.hstack((R, t))
M_l = np.hstack((np.eye(3, 3), np.zeros((3, 1))))

P_l = np.dot(K,  M_l)
P_r = np.dot(K,  M_r)
point_4d_hom = cv2.triangulatePoints(P_l, P_r, np.expand_dims(pts_l, axis=1), np.expand_dims(pts_r, axis=1))
point_4d = point_4d_hom / np.tile(point_4d_hom[-1, :], (4, 1))
point_3d = point_4d[:3, :].T

x, y, z = point_3d.T

#print("point_3d ",point_3d)

# E, mask = cv2.findEssentialMat(pts_r_norm, pts_l_norm, K, cv2.FM_RANSAC, 0.999, 1.0, None)
# print("E: ", E)
#
# points, R, t, mask = cv2.recoverPose(E, pts2, pts1, K)
# print("R: ", R)
# print("t: ", t)

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()

cv2.waitKey(0)
cv2.destroyAllWindows()
