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

# @file    wrapper.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief main file that calls all other sub functions

import sys

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import argparse
from scripts.LoadData import *
from scripts.bdl_adjst import *
from scripts.visibikity_matrx import *
from scripts.NonLinearPnP import *
from scripts.DrawCameras import camera_draw
from scripts.ExtractCameraPose import extract_cam_pose
from scripts.EssentialMatrixFromFundamentalMatrix import e_from_fundamental
from scripts.NonLinearTriangulation import *
from scripts.EstimateFundamentalMatrix import est_from_funda
from scripts.PnPRANSAC import *
from scripts.disambguate_camera_pose import *
from scripts.LinearTriangulation import *

K = np.array([[568.996140852,              0,   643.21055941],
              [0,              568.988362396,  477.982801038],
              [0,                          0,              1]])

W = np.array([[0, -1, 0],
              [1,  0, 0],
              [0,  0, 1]])


n_images = 6
img1 = 1
img2 = 4

Parser = argparse.ArgumentParser()
Parser.add_argument(
    '--DataPath', default="./Data/", help='Folder of Images')
Parser.add_argument(
    '--Visualize', default=False, help='Show correspondences')
Args = Parser.parse_args()
DataPath = Args.DataPath
visualize = Args.Visualize

x_coord_matrix, y_coord_matrix, M, Color = LoadData(DataPath)

M, outlier_indices = inlier_filter(x_coord_matrix, y_coord_matrix, M, n_images)

recon_bin = np.zeros((M.shape[0], 1))

X_3D = np.zeros((M.shape[0], 3))

opt = np.logical_and(M[:, img1 - 1], M[:, img2 - 1])

outlier_idx = np.where(np.logical_and(outlier_indices[:, img1 - 1],
                                      outlier_indices[:, img2 - 1]) == True)
indices, = np.where(opt == True)

rgb_list = Color[indices]

best_F = est_from_funda(np.float32(np.hstack((x_coord_matrix[indices, img1 - 1].reshape((-1, 1)),
                                                         y_coord_matrix[indices, img1 - 1].reshape((-1, 1))))),
                                   np.float32(np.hstack((x_coord_matrix[indices, img2 - 1].reshape((-1, 1)),
                                                         y_coord_matrix[indices, img2 - 1].reshape((-1, 1))))))

E = e_from_fundamental(best_F, K)
R_set, C_set = extract_cam_pose(E, K)
X_set = []

for n in range(0, 4):
    X1 = LinearTriangulation(K, np.zeros((3, 1)), np.identity(3),
                             C_set[n].T, R_set[n],
                             np.float32(np.hstack((x_coord_matrix[indices, img1 - 1].reshape((-1, 1)),
                                                   y_coord_matrix[indices, img1 - 1].reshape((-1, 1))))),
                             np.float32(np.hstack((x_coord_matrix[indices, img2 - 1].reshape((-1, 1)),
                                                   y_coord_matrix[indices, img2 - 1].reshape((-1, 1))))))
    X_set.append(X1)

X, R, C = disambguate_camera_pose(C_set, R_set, X_set)

recon_bin = np.zeros((M.shape[0], 1))
X_3D = np.zeros((M.shape[0], 3))
Visibility = np.zeros((M.shape[0], n_images))

X = NonLinearTriangulation(K, np.float32(np.hstack((x_coord_matrix[indices, img1 - 1].reshape((-1, 1)),
                                                    y_coord_matrix[indices, img1 - 1].reshape((-1, 1))))),
                           np.float32(np.hstack((x_coord_matrix[indices, img2 - 1].reshape((-1, 1)),
                                                 y_coord_matrix[indices, img2 - 1].reshape((-1, 1))))), X, np.eye(3),
                           np.zeros((3, 1)), R, C)

recon_bin[indices] = 1

X_3D[indices, :] = X

Visibility[indices, img1 - 1] = 1

Visibility[indices, img2 - 1] = 1

Cset = []
Rset = []

Cset.append(C)
Rset.append(R)

r_indx = [img1, img2]

for i in range(0, n_images):

    if (np.isin(r_indx, i)[0]):
        continue

    opt = np.logical_and(recon_bin, M[:, i].reshape((-1, 1)))

    indices, _ = np.where(opt == True)

    if (len(indices) < 8):
        continue

    x = np.transpose([x_coord_matrix[indices, i], y_coord_matrix[indices, i]])

    X = X_3D[indices, :]

    C, R = PnPRANSAC(X, x, K)

    C, R = NonLinearPnP(X, x, K, C, R)

    Cset.append(C)

    Rset.append(R)

    r_indx.append(i)

    Visibility[indices, i] = 1

    for j in range(0, len(r_indx) - 1):

        opt = np.logical_and(
            np.logical_and(1 - recon_bin, M[:, r_indx[j]].reshape(
                (-1, 1))), M[:, i].reshape((-1, 1)))

        indices, _ = np.where(opt == True)

        if (len(indices) < 8):
            continue

        x1 = np.hstack((x_coord_matrix[indices, r_indx[j]].reshape((-1, 1)),
                        y_coord_matrix[indices, r_indx[j]].reshape((-1, 1))))
        x2 = np.hstack((x_coord_matrix[indices, i].reshape((-1, 1)),
                        y_coord_matrix[indices, i].reshape((-1, 1))))

        X = LinearTriangulation(K, Cset[j], Rset[j], C, R, x1, x2)

        X_3D[indices, :] = X

        recon_bin[indices] = 1

        Visibility[indices, r_indx[j]] = 1

        Visibility[indices, j] = 1

    for o in range(len(X_3D)):
        if (X_3D[o, 2] < 0):
            Visibility[o, :] = 0
            recon_bin[o] = 0

    V_bundle = visibikity_matrx(Visibility, r_indx)

    point_indices, _ = np.where(recon_bin == 1)
    camera_indices = i * np.ones((len(point_indices), 1))

    points_2d = np.hstack((x_coord_matrix[point_indices, i].reshape((-1, 1)),
                           x_coord_matrix[point_indices, i].reshape((-1, 1))))

    Rset, Cset, X_3D = bdl_adjst(Cset, Rset, X_3D, K, points_2d,
                                        camera_indices, recon_bin,
                                        V_bundle)

ind, _ = np.where(recon_bin == 1)

X_3D = X_3D[ind, :]

Color = Color[ind, :]

ax = plt.axes(projection='3d')

ax.scatter3D(
    X_3D[:, 0], X_3D[:, 1], X_3D[:, 2], c=Color / 255.0,
    s=1)

ax.set_xlabel('x')

ax.set_ylabel('y')

ax.set_zlabel('z')

ax.set_xlim([-0.5, 1])

ax.set_ylim([-0.5, 1])

ax.set_zlim([0, 1.5])

plt.show()

plt.scatter(X_3D[:, 0], X_3D[:, 2], c=Color / 255.0, s=1)

camera_draw(C_set, R_set)

ax1 = plt.gca()

ax1.set_xlabel('x')

ax1.set_ylabel('z')

plt.show()