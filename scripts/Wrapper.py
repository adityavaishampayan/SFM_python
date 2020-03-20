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

# @file    Wrapper.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  Full pipeline of structure from motion


import sys

try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

import cv2
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import glob

from scripts.Frame import set_key_pts, set_descriptors, set_pose, set_3d_pts, get_detections, get_pose, get_3d_pts

from scripts.EstimateFundamentalMatrix import *
from scripts.GetInliersRANSAC import *
from scripts.EssentialMatrixFromFundamentalMatrix import *
from scripts.ExtractCameraPose import *
from scripts.DisambiguateCameraPose import *
from scripts.NonlinearTriangulation import *
from scripts.LinearPnP import *
from scripts.NonlinearPnP import *
from scripts.BuildVisibilityMatrix import *
from scripts.BundleAdjustment import *

K = np.array([[568.996140852, 0, 643.21055941],
              [0, 568.988362396, 477.982801038],
              [0, 0, 1]])

W = np.array([[0, -1, 0],
              [1, 0, 0],
              [0, 0, 1]])
