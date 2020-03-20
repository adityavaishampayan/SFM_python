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

# @file    Frame.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  Frame Class to store and retrieve all related information

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass


def linear_triangulation(camera_pose1: float, camera_pose2: float, key_pts1: float, key_pts2: float) -> float:
    """
    a function to estimate 3D points given camera poses and matching key point correspondences
    :param camera_pose1: (C1,R1)
    :param camera_pose2: (C2,R2)
    :param key_pts1: matching keypoints from first image
    :param key_pts2: matching keypoints from second image
    :return: 3D points estimated using linear triangulation
    """
    ...
