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

# @file    EstimateFundamentalMatrix.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  linearly estimates a fundamental matrix F, such that xT2Fx1=0.


import sys
import numpy as np

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


def get_fundamental_matrix(key_pts1: float, key_pts2: float) -> float:
    """
    A function to obtain a fundamental matrix given the matching keypoints by solving Ax = 0
    :param key_pts1: one set of Keypoints from one image
    :param key_pts2: another set of Keypoints from second image
    :return: Fundamental Matrix
    """
    n = key_pts1.shape[1]
    A = np.zeros((n, 9))
    for i in range(n):
        A[i] = [key_pts1[i, 0] * key_pts2[i, 0],
                key_pts1[i, 0] * key_pts2[i, 1],
                key_pts1[i, 0],
                key_pts1[i, 1] * key_pts2[i, 0],
                key_pts1[i, 1] * key_pts2[i, 1],
                key_pts1[i, 1],
                key_pts2[i, 0],
                key_pts2[i, 1],
                1]

    U, S, V = np.linalg.svd(A)
    F = V[-1].reshape(3, 3)

    U, S, V = np.linalg.svd(F)

    S[2] = 0

    F = np.dot(U, np.dot(np.diag(S), V))
    
    F = F/ F[2,2]
    return F