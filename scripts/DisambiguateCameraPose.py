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

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


def disambiguate_camera_pose(
        camera_pose1: float,
        camera_pose2: float,
        camera_pose3: float,
        camera_pose4: float,
        triangulated_points: float) -> float:
    """
    Given four camera pose configurations and their triangulated points return the unique camera pose by checking
    the cheirality condition
    :param camera_pose1: (C1,R1)
    :param camera_pose2: (C2,R2)
    :param camera_pose3: (C3,R3)
    :param camera_pose4: (C4,R4)
    :param triangulated_points: triangulated points obtained from linear triangulation
    :return: Unique camera pose
    """
    ...
