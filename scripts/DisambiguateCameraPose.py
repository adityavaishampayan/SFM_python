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

# @file    DisambiguateCameraPose.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
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


def DisambiguateCameraPose(Cset, Rset, Xset):
    """ Function to implement camera pose correction
    Args:
        Cset (TYPE): Set of calculated camera poses
        Rset (TYPE): Set of calculated Rotation matrices
        Xset (TYPE): 3D points
    Returns:
        TYPE: Corrected X, R_set, C_set
    """
    best = 0
    for i in range(4):

        #         Cset[i] = np.reshape(Cset[i],(-1,-1))
        N = Xset[i].shape[0]
        n = 0
        for j in range(N):
            if ((np.dot(Rset[i][2, :], (Xset[i][j, :] - Cset[i])) > 0)
                    and Xset[i][j, 2] >= 0):
                n = n + 1
        if n > best:
            C = Cset[i]
            R = Rset[i]
            X = Xset[i]
            best = n

    return X, R, C