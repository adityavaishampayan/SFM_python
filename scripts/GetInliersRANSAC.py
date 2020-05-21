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

# @file    GetInlierRANSANC.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  a function that estimates inlier correspondences using fundamental matrix based RANSAC.

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

import numpy as np
from EstimateFundamentalMatrix import EstimateFundamentalMatrix
import sys

sys.dont_write_bytecode = True


def GetInliersRANSAC(kpts1_matches, kpts2_matches, indices):
    """
    function to implement ransac for outlier rejection on key point matches and then obtaining refined Fundamental
    matrix
    :param kpts1_matches: one set of Keypoints from one image
    :param kpts2_matches: second set of Keypoints from second image
    :param indices: indices of the corresponding keypoints
    :return: refined Fundamental Matrix, inliers and their corresponding indices
    """

    num_of_matches = kpts1_matches.shape[0]
    Best_count = 0

    for iter in range(500):

        sampled_idx = np.random.randint(0, num_of_matches, size=8)
        ind = []
        in_a = []
        in_b = []
        F = EstimateFundamentalMatrix(kpts1_matches[sampled_idx, :], kpts2_matches[sampled_idx, :])
        new_set_inlier_count = 0

        for i in range(num_of_matches):
            error_1 = np.dot(np.append(kpts1_matches[i, :], 1), F.T)
            if abs(np.dot(error_1, np.append(kpts2_matches[i, :], 1).T)) < 0.005:
                in_a.append(kpts1_matches[i, :])
                in_b.append(kpts2_matches[i, :])
                ind.append(indices[i])
                new_set_inlier_count += 1

        if new_set_inlier_count > Best_count:
            Best_count = new_set_inlier_count
            best_F = F
            best_inliers_a = in_a
            best_inliers_b = in_b
            inlier_index = ind

    return best_F, np.array(best_inliers_a), np.array(best_inliers_b), np.array(inlier_index)
