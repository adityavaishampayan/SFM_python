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

## @file    frame.py
# @Author  Amrish Baskaran (amrish1222)
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief  Frame Class to store and retrieve all related information

import numpy as np

class Frame:

    # @brief Constructor for Class
    # @param  self- Object pointer
    # @return none
    def __init__(self, frameId):
        self.__frameId = frameId;
        self.__keyPts = []
        self.__descriptors = []
        self.__currentR = np.zeros((3,3))
        self.__currentT = np.zeros((3,1))
        self.__pts3d = []
        
    
    # @brief set the keypoint extracted by the detector
    # @param  self- Object pointer
    # @param  ptsList- List of np elements size(1,2)
    # @return none
    def setKeyPts(self, ptsList):
        self.__keyPts = ptsList
    
    # @brief set the descriptors extracted by the detector corresponding to keypts
    # @param  self- Object pointer
    # @param  ptsList- List of np elements size(1,2)
    # @return none
    def setDescriptors(self, descriptorList):
        self.__descriptors = descriptorList
        
    # @brief set the pose paramters
    # @param  self- Object pointer
    # @param  R- Rotation matrix
    # @param  T- Translation vector
    # @return none
    def setPose(self, R, T):
        self.__currentR = R
        self.__currentT = T
        
    # @brief set the 3D points associated with this camera Pose
    # @param  self- Object pointer
    # @param  ptsList- List of np elements size(1,3)
    # @return none
    def set3dPts(self, ptsList):
        self.__pts3d = ptsList
    
    # @brief return the keypoints and descriptors
    # @param  self- Object pointer
    # @return __keyPts,__descriptors
    def getDetections(self):
        return self.__keyPts, self.__descriptors
    
    # @brief return the pose associated with this pose
    # @param  self- Object pointer
    # @return __currentR, __currentT
    def getPose(self):
        return self.__currentR, self.__currentT

    # @brief returns 3d Pts associated with this camera pose
    # @param  self- Object pointer
    # @return __pts3d
    def get3dPts(self):
        return self.__pts3d
