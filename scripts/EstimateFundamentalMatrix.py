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

# @file    EstimateFundamentalMatrix.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  linearly estimates a fundamental matrix F, such that xT2Fx1=0.


import sys
import numpy as np

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


def est_from_funda(key_pts1, key_pts2):
    """
        A function to obtain a fundamental matrix given the matching keypoints by solving Ax = 0
        :param key_pts1: one set of Keypoints from one image
        :param key_pts2: another set of Keypoints from second image
        :return: Fundamental Matrix
        """

    # determining the shape of key points1
    kpts1_shape = key_pts1.shape[0]

    # determining the shape of keypoints 2
    kpts2_shape = key_pts2.shape[0]

    kpts2_shape = kpts1_shape

    A = []
    B = np.ones((kpts1_shape, 1))

    s1_a_array = np.array([[kpts1_shape / np.sum(((key_pts1[:, 0] - np.sum(key_pts1[:, 0]) / kpts1_shape)**2 + (key_pts1[:, 1] - np.sum(key_pts1[:, 1]) / kpts1_shape)**2)**(1 / 2)), 0, 0],
                           [0, kpts1_shape / np.sum(((key_pts1[:, 0] - np.sum(key_pts1[:, 0]) / kpts1_shape)**2 + (key_pts1[:, 1] - np.sum(key_pts1[:, 1]) / kpts1_shape)**2)**(1 / 2)), 0],
                           [0, 0,  1]])

    s1_b_array = np.array([[1, 0, -(np.sum(key_pts1[:, 0]) / kpts1_shape)],
                           [0, 1, -(np.sum(key_pts1[:, 1]) / kpts1_shape)],
                           [0, 0,     1]])

    s2_a_array = np.array([[kpts1_shape / np.sum(((key_pts2[:, 0] - np.sum(key_pts2[:, 0]) / kpts2_shape)**2 + (key_pts2[:, 1] - np.sum(key_pts2[:, 1]) / kpts2_shape)**2)**(1 / 2)), 0, 0],
                           [0, kpts1_shape / np.sum(((key_pts2[:, 0] - np.sum(key_pts2[:, 0]) / kpts2_shape)**2 + (key_pts2[:, 1] - np.sum(key_pts2[:, 1]) / kpts2_shape)**2)**(1 / 2)), 0],
                           [0,  0, 1]])

    s2_b_array = np.array([[1, 0, -(np.sum(key_pts2[:, 0]) / kpts2_shape)],
                           [0, 1, -(np.sum(key_pts2[:, 1]) / kpts2_shape)],
                           [0, 0,     1]])

    T_a = np.dot(s1_a_array, s1_b_array)
    key_pts1 = np.array(key_pts1.T)
    key_pts1 = np.append(key_pts1, B)
    key_pts1 = np.reshape(key_pts1, (3, kpts1_shape))
    key_pts1 = np.dot(T_a, key_pts1)
    key_pts1 = key_pts1.T

    T_b = np.dot( s2_a_array, s2_b_array)
    key_pts2 = np.array(key_pts2.T)
    key_pts2 = np.append(key_pts2, B)
    key_pts2 = np.reshape(key_pts2, (3, kpts1_shape))
    key_pts2 = np.dot(T_b, key_pts2)
    key_pts2 = key_pts2.T

    for i in range(kpts1_shape):
        u_a = key_pts1[i, 0]
        u_b = key_pts2[i, 0]

        v_a = key_pts1[i, 1]
        v_b = key_pts2[i, 1]
        A.append([
            u_a * u_b,
            v_a * u_b,
            u_b, u_a * v_b,
            v_a * v_b,
            v_b,
            u_a,
            v_a,
            1
        ])

    _, _, V = np.linalg.svd(A)
    F = V[-1]

    F = np.reshape(F, (3, 3)).T
    F = np.dot(T_a.T, F)
    F = np.dot(F, T_b)

    F = F.T
    U, S, V = np.linalg.svd(F)
    S = np.array([[S[0], 0, 0],
                  [0, S[1], 0],
                  [0,    0, 0]])

    F = np.dot(U, S)
    F = np.dot(F, V)

    F = F / F[2, 2]

    return F
