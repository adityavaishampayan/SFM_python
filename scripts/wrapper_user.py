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

# @file    Wrapper_user.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  Full pipeline of structure from motion using user functions
from scripts.ransac_eight_point import ransac_eight_points
from scripts.EssentialMatrixFromFundamentalMatrix import get_essential_matrix
from scripts.ExtractCameraPose import extract_camera_pose
from scripts.LinearTriangulation import LinearTriangulation
from scripts.DisambiguateCameraPose import DisambiguateCameraPose

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except :
    pass

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import math
import pickle
path = r'/home/aditya/SFM_python/Dataset/Data/'

with open(path + 'matches1_2.data', 'rb') as filehandle:
    # read the data as binary data stream
    matches1_2 = pickle.load(filehandle)

K = np.array([[568.996140852, 0, 643.21055941],
              [0, 568.988362396, 477.982801038],
              [0, 0, 1]])

W = np.array([[0, -1, 0],
              [1, 0, 0],
              [0, 0, 1]])

list1 = []
list2 = []

for pair in matches1_2:
    list1.append((int(float(pair[0][0])),int(float(pair[0][1]))))
    list2.append((int(float(pair[1][0])),int(float(pair[1][1]))))

img1 = cv2.imread(path+"1.jpg")
img2 = cv2.imread(path+"2.jpg")

for i in list1:
    res1 = cv2.circle(img1, i, 2, (0, 0, 255), -1)

for i in list2:
    res2 = cv2.circle(img2, i, 2, (0, 0, 255), -1)

joint_list = zip(list1, list2)

BF = ransac_eight_points(joint_list)

E = get_essential_matrix(BF, K)

#print(E)

R_set,C_set = extract_camera_pose(E,W)

# print("rset", R_set)
# print("cset", C_set)

X_set = []
color = ['r', 'g', 'b', 'k']

for n in range(0, 4):
    X1 = LinearTriangulation(K, np.zeros((3, 1)), np.identity(3),
                             C_set[n].T, R_set[n], np.float32(list1),
                             np.float32(list2))
    X_set.append(X1)

X, R, C = DisambiguateCameraPose(C_set, R_set, X_set)

print(X)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_values = []
y_values = []
z_values = []

for i in X_set:
    x_values.append(X_set[0])
    y_values.append(X_set[1])
    z_values.append(X_set[2])

ax.scatter(x_values, z_values, c='r', marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()

# plt.scatter(X[:, 0], X[:, 2], c='r', s=4)
# ax = plt.gca()
# ax.set_xlabel('x')
# ax.set_ylabel('y')
#
# ax.set_xlim([-0.5, 0.5])
# ax.set_ylim([-0.1, 2])
#
# plt.show()


#
# cv2.imshow("res1",res1)
# cv2.imshow("res2",res2)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

#BF, inliers = ransac_eight_points(jointlist)