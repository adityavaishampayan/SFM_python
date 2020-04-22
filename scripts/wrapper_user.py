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

print(matches1_2)

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

cv2.imshow("res1",res1)
cv2.imshow("res2",res2)
cv2.waitKey(0)
cv2.destroyAllWindows()

#BF, inliers = ransac_eight_points(jointlist)