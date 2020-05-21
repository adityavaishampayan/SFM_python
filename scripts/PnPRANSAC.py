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

# @file    PnpRansac.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass
from tqdm import tqdm
import LinearPnP as LPnP
import random
import numpy as np


def proj3Dto2D(3d_x_points, calibration_matrix, C, R):
    """
    Projecting points from 3 dimensional space to 2 dimensional space
    :param 3d_x_points: 3d points
    :param calibration_matrix: calibration matrix
    :param C: translation vector
    :param R: rotation matrix
    :return: projected 2d points
    """


    P = np.dot(np.dot(calibration_matrix, R), np.hstack((np.identity(3), -C.reshape(-1, 1))))

    pose_3d_dot = (np.dot(P[2, :], np.vstack((3d_x_points.reshape(-1, 1), 1)))).T

    x_2d_points = np.hstack((((np.dot(P[0, :], np.vstack((3d_x_points.reshape(-1, 1), 1)))).T / pose_3d_dot), ((np.dot(P[1, :], np.vstack((3d_x_points.reshape(-1, 1), 1)))).T / pose_3d_dot)))

    return x_2d_points


def PnPRANSAC(X, x, calibration_matrix):


    new_r = np.identity(3)
    threshold = 5
    cnt = 0
    new_c = np.zeros((3, 1))
    homogenous_x = LPnP.convertHomogeneouos(x)

    for i in tqdm(range(500)):
        ran_idx = random.sample(range(x.shape[0]), 6)

        alpha =X[ran_idx][:]

        beta =  x[ran_idx][:]

        C, R = LPnP.LinearPnP(alpha, beta, calibration_matrix)

        S = []

        homo_alpha = homogenous_x[j][:]

        for j in range(x.shape[0]):
            reproj = proj3Dto2D(homo_alpha, calibration_matrix, C, R)

            e = np.sqrt(np.square((homogenous_x[j, 0]) - reproj[0]) + np.square((homogenous_x[j,1] - reproj[1])))

            if e < threshold:
                S.append(j)

        countS = len(S)

        if (countS == x.shape[0]):
            break

        if (cnt < countS):
            cnt = countS
            new_r = R
            new_c = C

    return new_c, new_r
