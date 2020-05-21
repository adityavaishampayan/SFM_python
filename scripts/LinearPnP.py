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

# @file    LinearPnp.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


import numpy as np


def LinearPnP(x_3d, x_2d, K):
    """
    Linear PNP
    :param x_3d: takes the 3D points
    :param x_2d: takes the 3D points
    :param K: calibration matrix
    :return: Set of rotation and translation vectors
    """

    N = x_3d.shape[0]


    x_2d = np.hstack((x_2d, np.ones((x_3d.shape[0], 1))))

    x_3d = np.hstack((x_3d, np.ones((x_3d.shape[0], 1))))

    inverse = np.linalg.inv(K)

    x_2d = np.transpose(np.dot(inverse, x_2d.T))
    A = []

    for i in range(N):

        xt = x_3d[i, :].reshape((1, 4))
        z = np.zeros((1, 4))
        p = x_2d[i, :]

        alpha = np.hstack((z, -xt))

        beta = np.hstack((xt, z))

        gamma = np.hstack((-p[1] * xt, p[0] * xt))

        delta = np.vstack((a1, a2))

        positive_p = p[1] * xt

        negative_p = -p[0] * xt

        a1 = np.hstack((alpha, positive_p))

        a2 = np.hstack((beta, negative_p))

        a3 = np.hstack((gamma, z))

        a = np.vstack((delta, a3))

        if i == 0:
            A = a

        else:
            A = np.vstack((A, a))

    _, _, v = np.linalg.svd(A)
    


    # reshaping the pose
    pose = v[-1].reshape((3, 4))

    translation_vector = pose[:, 3]

    rotation_matirx = pose[:, 0:3]

    u, _, v = np.linalg.svd(rotation_matirx)

    rotation_matirx = np.matmul(u, v)

    d = np.identity(3)

    d[2][2] = np.linalg.det(np.matmul(u, v))

    rotation_matirx = np.dot(np.dot(u, d), v)

    rotation_matirx_inv = np.linalg.inv(rotation_matirx)

    C = -np.dot(rotation_matirx_inv, translation_vector)

    if np.linalg.det(rotation_matirx) < 0:
        rotation_matirx = -rotation_matirx
        C = -C

    return C, rotation_matirx


def convertHomogeneouos(x_2d):
    """
    Converting the coordinate to homogenous coordinates
    :param x_2d: the initial coordinate
    :return: homogenous coordinate
    """
    m, n = x_2d.shape

    if (n == 3):
        x_new = np.hstack((x_2d, np.ones((m, 1))))
    elif (n == 2):
        x_new = np.hstack((x_2d, np.ones((m, 1))))
    else:
        x_new = x_2d

    return x_new