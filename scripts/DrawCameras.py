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

# @file    draw_camera.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  Given four camera pose configurations and their triangulated points return the unique camera pose by checking
# the cheirality condition

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

import numpy as np
import matplotlib.pyplot as plt
import math
import sys

# Checks if a matrix is a valid rotation matrix.


def r_to_euler(R):
    """
    euler angles from rotation matrices
    :param R: rot_matrix
    :return: euler angles
    """
    if not (math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]) < 1e-6):
        beta = math.atan2(-R[2, 0], math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
        gamma = math.atan2(R[1, 0], R[0, 0])
        alpha = math.atan2(R[2, 1], R[2, 2])
        return np.array([alpha, beta, gamma])
    else:
        beta = math.atan2(-R[2, 0], math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]))
        alpha = math.atan2(-R[1, 2], R[1, 1])
        gamma = 0
        return np.array([alpha, beta, gamma])



def r_matrix_check(matrix):
    """
    check if the given matrix is a rotation matrix or not
    :param matrix: input matrix
    :return: norm of the matrix
    """
    n = np.linalg.norm(np.identity(3, dtype=matrix.dtype) - np.dot(np.transpose(matrix), matrix))
    return n < 1e-6


def camera_draw(C_set, R_set):
    """
    draw camera positions
    :param C_set: camera centers
    :param R_set: rotation matrix center
    :return: none, it plots all the camera centers
    """

    for i in range(0, len(C_set)):
        R1 = r_to_euler(R_set[i])

        a = C_set[i][0]
        b = C_set[i][2]
        c = (3, 0, int(np.rad2deg(R1)[1]))
        d = 15

        plt.plot(a, b, c, d, linestyle='None')
