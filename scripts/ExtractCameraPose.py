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

# @file    ExtractCameraPose.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  Estimate Camera Pose from Essential Matrix


import sys
import numpy as np

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


K = np.array([[568.996140852,              0,   643.21055941],
              [0,              568.988362396,  477.982801038],
              [0,                          0,              1]])


def extract_cam_pose(E, K):
    """
    function to obtain Rotation and Translation
    :param essential_matrix: a 3x3 matrix
    :param w_matrix: a 3x3 matrix that is multiplied with essential matrix
    :return: Rotation and translation matrices
    """
    U, S, V_T = np.linalg.svd(E)

    R = []
    C = []

    # rotation matrices
    R.append(np.dot(U, np.dot(np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]]), V_T)))
    R.append(np.dot(U, np.dot(np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]]), V_T)))
    R.append(np.dot(U, np.dot(np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]]).T, V_T)))
    R.append(np.dot(U, np.dot(np.array([[0, -1, 0],
                  [1,  0, 0],
                  [0,  0, 1]]).T, V_T)))

    #  translation matrices
    C.append(U[:, 2])
    C.append(-U[:, 2])
    C.append(U[:, 2])
    C.append(-U[:, 2])

    for i in range(4):
        if (np.linalg.det(R[i]) < 0):
            R[i] = -R[i]
            C[i] = -C[i]

    return R, C
