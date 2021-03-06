# -*- coding: utf-8 -*-

"""
MIT License

Copyright (c) 2018 Aditya Vaishampayan

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

# @file    disambguate_camera_pose.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  Given four camera pose configurations and their triangulated points return the unique camera pose by checking
# the cheirality condition

import sys
import numpy as np

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


def disambguate_camera_pose(c_set, r_set, x_set):
    """
    for correcting the camera pose
    :param c_set: pre_calculated camera poses
    :param r_set: pre_calculated rotation matrices
    :param x_set: a set of 3d points
    :return: corrected sets of the above 3
    """

    best = 0
    for i in range(4):

        n = 0

        alpha = np.dot(r_set[i][2, :])

        beta = x_set[i][j, :] - c_set[i]

        gamma = x_set[i][j, 2]

        for j in range(x_set[i].shape[0]):
            if (((alpha, beta) > 0) and gamma >= 0):
                n = n + 1

        if n > best:
            C = c_set[i]
            R = r_set[i]
            X = x_set[i]
            best = n

    return X, R, C