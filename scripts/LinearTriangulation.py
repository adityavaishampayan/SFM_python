# -*- coding: utf-8 -*-

"""
MIT License

Copyright (c) 2020 Aditya Vaishampayan

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

# @file    Linear Triangulation.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  Frame Class to store and retrieve all related information


import sys
import numpy as np

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

def linear_triangulation(K,camera_pose1: float, rotation_matrix1: float, camera_pose2: float, , rotation_matrix2: float,
                         key_pts1: float, key_pts2: float) -> float:
    """
    a function to estimate 3D points given camera poses and matching key point correspondences
    :param camera_pose1: (camera_pose1,rotation_matrix1)
    :param camera_pose2: (camera_pose2,rotation_matrix2)
    :param key_pts1: matching keypoints from first image
    :param key_pts2: matching keypoints from second image
    :return: 3D points estimated using linear triangulation
    """

    X = np.zeros((key_pts1.shape[0], 3))

    for i in range(key_pts1.shape[0]):

        A = np.vstack((np.dot(skew(np.hstack((key_pts1, np.ones((key_pts1.shape[0], 1))))[i, :]), np.dot(K, np.dot(rotation_matrix1, np.hstack((np.identity(3), -np.reshape(camera_pose1, (3, 1))))))), np.dot(skew(np.hstack((key_pts2, np.ones((key_pts1.shape[0], 1))))[i, :]), np.dot(K, np.dot(rotation_matrix2, np.hstack((np.identity(3), -np.reshape(camera_pose2, (3, 1)))))))))

        _, _, v = np.linalg.svd(A)

        x = v[-1] / v[-1, -1]

        x = np.reshape(x, (len(x), -1))

        X[i, :] = x[0:3].T

    return X


def skew(x):
    return np.array([[0, -x[2], x[1]], [x[2], 0, x[0]], [x[1], x[0], 0]])


