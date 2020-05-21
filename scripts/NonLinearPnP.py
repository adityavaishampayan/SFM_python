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
# @brief  a function that refines the camera pose that minimizes reprojection error


import sys
import numpy as np

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

import numpy as np
import scipy.optimize as opt
from scipy.spatial.transform import Rotation as Rscipy


def reprojError(CQ, K, X, x):
    """
    calculating the reprojection error
    :param K: calibration matrix
    :param X: 3d poiints
    :param x: 2d points
    :return: reprojection error
    """

    X = np.hstack((X, np.ones((X.shape[0], 1))))

    P = np.dot(np.dot(K, (Rscipy.from_quat([CQ[3:7][0], CQ[3:7][1], CQ[3:7][2], CQ[3:7][3]])).as_dcm()), np.hstack((np.identity(3), -CQ[0:3].reshape(-1, 1))))

    e = x[:, 0] - (np.dot(P[0, :], X.T)).T / (np.dot(P[2, :], X.T)).T + x[:, 1] - (np.dot(P[1, :], X.T)).T / (np.dot(P[2, :], X.T)).T

    return sum(e)


def NonLinearPnP(X, x, K, C0, R0):

    optimized_param = opt.least_squares(
        fun=reprojError, method="dogbox", x0=[C0[0], C0[1], C0[2], ((Rscipy.from_dcm(R0)).as_quat())[0], ((Rscipy.from_dcm(R0)).as_quat())[1], ((Rscipy.from_dcm(R0)).as_quat())[2], ((Rscipy.from_dcm(R0)).as_quat())[3]], args=[K, X, x])
    Cnew = optimized_param.x[0:3]
    Rnew = (Rscipy.from_quat([(optimized_param.x[3:7])[0], (optimized_param.x[3:7])[1], (optimized_param.x[3:7])[2], (optimized_param.x[3:7])[3]])).as_dcm()

    return Cnew, Rnew
