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

# @file    NonlinearTriangulation.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  refine the locations of the 3D points that minimizes re-projection error

import sys
import numpy as np
import scipy.optimize as opt

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

def minimizer(init, K, x1, x2, R1, translation_vector_1, R2, translation_vector_2):
    """
    Minimizer function
    :param init:
    :param K: calibration matrix
    :param x1: set of points
    :param x2: set of points
    :param R1: rotation matrix 1
    :param translation_vector_1: translation vector
    :param R2: rotation matrix 2
    :param translation_vector_2: translation vector 2
    :return: error
    """

    X = np.reshape(init, (x1.shape[0], 3))

    translation_vector_2 = np.reshape(translation_vector_2, (3, -1))

    X = np.hstack((X, np.ones((x1.shape[0], 1))))

    u_1 = np.divide(np.dot(np.dot(K, np.dot(R1, np.hstack((np.identity(3), -translation_vector_1))))[0, :], X.T).T, np.dot(np.dot(K, np.dot(R1, np.hstack((np.identity(3), -translation_vector_1))))[2, :], X.T).T)

    v_1 = np.divide(np.dot(np.dot(K, np.dot(R1, np.hstack((np.identity(3), -translation_vector_1))))[0, :], X.T).T, np.dot(np.dot(K, np.dot(R1, np.hstack((np.identity(3), -translation_vector_1))))[2, :], X.T).T)

    u_2 = np.divide(np.dot(np.dot(K, np.dot(R2, np.hstack((np.identity(3), -translation_vector_2))))[0, :], X.T).T, np.dot(np.dot(K, np.dot(R2, np.hstack((np.identity(3), -translation_vector_2))))[2, :], X.T).T)

    v_2 = np.divide(np.dot(np.dot(K, np.dot(R2, np.hstack((np.identity(3), -translation_vector_2))))[0, :], X.T).T, np.dot(np.dot(K, np.dot(R2, np.hstack((np.identity(3), -translation_vector_2))))[2, :], X.T).T)

    e_1_a = x1[:, 0] - u_1

    e_1_b = x1[:, 1] - v_1

    e_2_a = x2[:, 0] - u_2

    e_2_b = x2[:, 1] - v_2

    final_error = e_1_a + e_1_b + e_2_a + e_2_b

    return sum(final_error)

def NonLinearTriangulation(K, x1, x2, X_init, R1, translation_vector_1, R2, translation_vector_2):
    """
    Given two camera poses and linearly triangulated points, X, refine the locations of the 3D points that minimizes
    re-projection error
    :param K: calibration matrix
    :param x1: points in image 1
    :param x2: points in image 2
    :param X_init: initial estimation after triangulation
    :param R1: rotation matrix 1
    :param translation_vector_1: translation vector 1
    :param R2: rotation matrix 2
    :param translation_vector_2: translation vector 2
    :return: triangulated points
    """

    x1_shape = x1.shape[0]


    X = np.zeros((x1_shape, 3))

    init = X_init.flatten()

    optimized_params = opt.least_squares(
        fun=minimizer,
        x0=init,
        method="dogbox",
        args=[K, x1, x2, R1, translation_vector_1, R2, translation_vector_2])

    X = np.reshape(optimized_params.x, (x1_shape, 3))

    return X


