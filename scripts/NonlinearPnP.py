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

# @file    NonlinearPnP.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  a function that refines the camera pose that minimizes reprojection error

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass


def non_linear_pnp(correspondences_2d_3d: float, camera_pose: float) -> float:
    """
    given 3D-2D correspondences, and linearly estimated camera pose, (C,R), refine the camera pose that minimizes
    re-projection error
    :param correspondences_2d_3d: 2D to 3D point correspondences
    :param camera_pose: linearly estimated camera pose (C,R)
    :return: camera pose with minimized re-projection error
    """
    ...
