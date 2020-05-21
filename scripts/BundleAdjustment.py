# not ready
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

# @file    bdl_adjst.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @copyright  MIT
# @brief   refining the camera poses and 3D pts by minimizing reprojection error

import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rscipy
from scipy.sparse import lil_matrix
from scipy.optimize import least_squares

# noinspection PyBroadException
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except BaseException:
    pass


def rot_pts(pts, rotation_vector):
    """
    rotating pts by a given vector
    :param pts: a set of pts to rot_pts
    :param rotation_vector: rotation vector
    :return: rotated pts
    """

    with np.errstate(invalid='ignore'):
        v = rotation_vector / np.linalg.norm(rotation_vector, axis=1)[:, np.newaxis]
        v = np.nan_to_num(v)

    return np.cos(np.linalg.norm(rotation_vector, axis=1)[:, np.newaxis]) * pts + np.sin(np.linalg.norm(rotation_vector, axis=1)[:, np.newaxis]) * np.cross(
        v, pts) + np.sum(pts * v, axis=1)[:, np.newaxis] * (1 - np.cos(np.linalg.norm(rotation_vector, axis=1)[:, np.newaxis])) * v


def project(pts, c_param):
    """
    a function for 3D to 2D conversion
    :param pts: points to be projected
    :param c_param: calibration matrix
    :return: projected 3d points onto 2 images
    """

    proj_pts = -(rot_pts(pts, c_param[:, :3]) + c_param[:, 3:6])[:, :2] / (rot_pts(pts, c_param[:, :3]) + c_param[:, 3:6])[:, 2, np.newaxis]

    proj_pts *= (1 + c_param[:, 7] * np.sum(proj_pts**2, axis=1) + c_param[:, 8] * np.sum(proj_pts**2, axis=1)**2 * c_param[:, 6])[:, np.newaxis]
    return proj_pts


def trial(params, n_cameras, n_points, cam_idxs, point_indices, points_2d):

    return (project((params[n_cameras * 9:].reshape((n_points, 3)))[point_indices], params[:n_cameras * 9].reshape((
        n_cameras, 9))[cam_idxs]) - points_2d).ravel()




def sparsity_adjst_bundle(n_cameras, n_points, cam_idxs,
                               point_indices):

    A = lil_matrix((cam_idxs.size * 2, n_cameras * 9 + n_points * 3), dtype=int)

    for s in range(9):
        A[2 * np.arange(cam_idxs.size), cam_idxs * 9 + s] = 1
        A[2 * np.arange(cam_idxs.size) + 1, cam_idxs * 9 + s] = 1

    for s in range(3):
        A[2 * np.arange(cam_idxs.size), n_cameras * 9 + point_indices * 3 + s] = 1
        A[2 * np.arange(cam_idxs.size) + 1, n_cameras * 9 + point_indices * 3 + s] = 1

    return A


def bdl_adjst(Cset, Rset, X, K, points_2d, cam_idxs, recon_bin,
                     V_bundle):

    c_param = []
    point_indices, _ = np.where(recon_bin == 1)
    V = V_bundle[point_indices, :]
    pts_3d = X[point_indices, :]
    for C0, R0 in zip(Cset, Rset):
        params = [((Rscipy.from_dcm(R0)).as_rotvec())[0],
                  ((Rscipy.from_dcm(R0)).as_rotvec())[1],
                  ((Rscipy.from_dcm(R0)).as_rotvec())[2],
                  C0[0],
                  C0[1],
                  C0[2],
                  K[1, 1], 0, 0]
        c_param.append(params)

    n_cameras = (np.reshape(c_param, (-1, 9))).shape[0]

    n_points = pts_3d.shape[0]

    opt = True

    if (opt):
        A = sparsity_adjst_bundle(n_cameras, n_points, cam_idxs,
                                       point_indices)

        x0 = np.hstack((c_param.ravel(), pts_3d.ravel()))

        res = least_squares(
            trial,
            x0,
            jac_sparsity=sparsity_adjst_bundle(n_cameras, n_points, cam_idxs,
                                       point_indices),
            verbose=2,
            x_scale='jac',
            ftol=1e-4,
            method='trf',
            args=((np.reshape(c_param, (-1, 9))).shape[0],
                  pts_3d.shape[0],
                  cam_idxs,
                  point_indices,
                  points_2d))

        parameters = res.x

        X = np.reshape(parameters[c_param.size:], (n_points, 3))

        for i in range(n_cameras):
            ((Rscipy.from_dcm(R0)).as_rotvec())[0] = np.reshape(parameters[0:c_param.size], (n_cameras, 9))[i, 0]
            ((Rscipy.from_dcm(R0)).as_rotvec())[1] = np.reshape(parameters[0:c_param.size], (n_cameras, 9))[i, 1]
            ((Rscipy.from_dcm(R0)).as_rotvec())[2] = np.reshape(parameters[0:c_param.size], (n_cameras, 9))[i, 2]

            Rset[i] = (Rscipy.from_rotvec([((Rscipy.from_dcm(R0)).as_rotvec())[0], ((Rscipy.from_dcm(R0)).as_rotvec())[1],((Rscipy.from_dcm(R0)).as_rotvec())[2]])).as_dcm()

            Cset[i] = [np.reshape(parameters[0:c_param.size], (n_cameras, 9))[i, 2],
                       np.reshape(parameters[0:c_param.size], (n_cameras, 9))[i, 2],
                       np.reshape(parameters[0:c_param.size], (n_cameras, 9))[i, 6]]

    return Rset, Cset, X
