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

# @file    ransac_eight_points.py
# @Author  Aditya Vaishampayan (adityavaishampayan)
# @Author  Amrish Baskaran (amrish1222)
# @copyright  MIT
# @brief  Estimates the inliers and fundamental matrix using Zhang's 8 point algorithm.


def ransac_eight_points(kpts_match):
    grid1 = [[[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []],
             [[], [], [], [], [], [], [], []]]

    resolution_X = 160
    resolution_Y = 120

    for index, kpts1, kpts2 in kpts_match:
        x_cell = int(kpts1[0] / resolution_X)
        y_cell = int(kpts1[1] / resolution_Y)
        grid1[x_cell][y_cell].append((kpts1[0], kpts1[1], kpts2[0], kpts1[1]))

    best_inliers = np.array([])
    non_empty_cells = []
    count = 0
    best_F = np.array([])

    for i in range(8):
        for j in range(8):
            if len(grid1[i][j]) != 0:
                non_empty_cells.append(grid1[i][j])

    while (count <= 100):

        eight_points = []

        eight_cells = random.sample(non_empty_cells, k=8)

        # obtaining eight points list1
        for cell in eight_cells:
            a = random.choice(cell)
            eight_points.append(a)

        points_img1 = []
        points_img2 = []
        for x1, y1, x2, y2 in eight_points:
            points_img1.append((x1, y1))
            points_img2.append((x2, y2))

        points_img1 = np.array(points_img1)
        points_img2 = np.array(points_img2)

        F = getFundamentalMatrix(points_img1.T, points_img2.T)

        inliers = []

        for index, kpts1, kpts2 in kpts_match:

            x_1 = kpts1[0]
            y_1 = kpts1[1]
            x_2 = kpts2[0]
            y_2 = kpts2[1]

            d1 = np.dot(F, np.array([x_1, y_1, 1]))
            d2 = np.dot(F.T, np.array([x_1, y_1, 1]))
            error = np.linalg.norm(np.abs(np.dot(np.array([x_2, y_2, 1]).T, d1))) / np.sqrt(
                np.dot(d1.T, d1) + np.dot(d2.T, d2))

            if error <= 0.5:
                inliers.append((x_1, y_1, x_2, y_2))

        if len(inliers) > len(best_inliers):
            best_inliers = np.array([])
            best_F = np.array([])
            best_inliers = np.array(inliers)
            best_F = F

        count += 1

    return best_F, best_inliers