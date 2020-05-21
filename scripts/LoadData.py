"""File to load data from the given files
"""
import numpy as np
from GetInliersRANSAC import GetInliersRANSAC
import sys

sys.dont_write_bytecode = True

def LoadData(path="Data/"):
    """Main function to load data from files

    Args:
        path (str, optional): Path for input text files

    Returns:
        Mx: List of x coordinates of points
        m_y: List of y coordinates of points
        M: Binary List of features w.r.t. images
        Color: List of RGB values of each feature
    """
    num_images = 6
    Mx = []
    m_y = []
    M = []
    Color = []
    for i in range(1, num_images):
        mx = []
        m_y = []
        m = []
        for j in range(i + 1, num_images + 1):

            x_list, y_list, binary_list, rgb_list = FindCorrespondence(
                i, j, path)

            if (j == i + 1):
                mx = x_list
                m_y = y_list
                m = binary_list

            else:
                mx = np.hstack((mx, x_list[:, 1].reshape((-1, 1))))
                m_y = np.hstack((m_y, y_list[:, 1].reshape((-1, 1))))
                m = np.hstack((m, binary_list[:, 1].reshape((-1, 1))))

        if (i == 1):
            Mx = mx
            m_y = m_y
            M = m
            Color = rgb_list

        else:

            mx = np.hstack((np.zeros((mx.shape[0], i - 1)), mx))
            m_y = np.hstack((np.zeros((m_y.shape[0], i - 1)), m_y))
            m = np.hstack((np.zeros((m.shape[0], i - 1)), m))
            assert Mx.shape[1] == mx.shape[1], "SHape not matched"
            Mx = np.vstack((Mx, mx))
            assert m_y.shape[1] == m_y.shape[1], "Shape m_y not matched"
            m_y = np.vstack((m_y, m_y))
            M = np.vstack((M, m))
            Color = np.vstack((Color, rgb_list))
    return Mx, m_y, M, Color


def FindCorrespondence(a, b, database_path):
    """To extract corrospondance between 2 images from given file

    Args:
        a (int): First Image Number
        b (int): Second Image Number
        database_path (str): path to read file

    Returns:
        list: List of matching points: x,y, binary, rgb
    """
    matching_list = []
    if (1 <= a <= 6):
        with open(database_path + "matching" + str(a) + ".txt") as f:
            line_no = 1
            for line in f:
                if line_no == 1:
                    line_no += 1
                    nfeatures = line[11:15]
                    nfeatures = int(nfeatures)

                else:
                    matching_list.append(line.rstrip('\n'))

    else:
        print("First image argument Number not found")

    final_list = []
    for i in range(0, len(matching_list)):
        current_row = matching_list[i]
        splitStr = current_row.split()
        current_row = []
        for j in splitStr:
            current_row.append(float(j))
        final_list.append(np.transpose(current_row))

    rgb_list = []
    x_list = []
    y_list = []
    binary_list = []
    for i in range(0, len(final_list)):
        rgb_row = []
        x_row = []
        y_row = []
        binary_row = []
        current_row = final_list[i]
        current_row = current_row[1:len(current_row)]

        res = np.where(current_row == b)

        x_row.append(current_row[3])
        y_row.append(current_row[4])
        binary_row.append(1)
        rgb_row.append(current_row[0])
        rgb_row.append(current_row[1])
        rgb_row.append(current_row[2])

        if (len(res[0]) != 0):
            index = res[0][0]
            x_row.append(current_row[index + 1])
            y_row.append(current_row[index + 2])
            binary_row.append(1)

        else:
            x_row.append(0)
            y_row.append(0)
            binary_row.append(0)

        if (len(x_row) != 0):
            x_list.append(np.transpose(x_row))
            y_list.append(np.transpose(y_row))
            binary_list.append(np.transpose(binary_row))
            rgb_list.append(np.transpose(rgb_row))

    return np.array(x_list), np.array(y_list), np.array(binary_list), np.array(
        rgb_list)


def inlier_filter(m_x, m_y, M, num_images):
    """
    To obtain a filtered list of inliers
    :param m_x: Matrix of x coordinates
    :param m_y: matrix of y coordinates
    :param M: binary list of features
    :param num_images: number of images
    :return: outlier_indices
    """

    outlier_indices = np.zeros(M.shape)
    for i in range(1, num_images):
        for j in range(i + 1, num_images + 1):
            img1 = i
            img2 = j

            output = np.logical_and(M[:, img1 - 1], M[:, img2 - 1])
            indices, = np.where(output == True)

            if (len(indices) < 8):
                continue

            m_x_image1 = m_x[indices, img1 - 1].reshape((-1, 1))

            m_y_image1 = m_y[indices, img1 - 1].reshape((-1, 1))

            pts1 = np.hstack(( m_x_image1, m_y_image1))

            m_x_image2 = m_x[indices, img2 - 1].reshape((-1, 1))

            m_y_image2 = m_y[indices, img2 - 1].reshape((-1, 1))

            pts2 = np.hstack((m_x_image2, m_y_image2))

            _, inliers_a, inliers_b, inlier_index = GetInliersRANSAC(np.float32(pts1), np.float32(pts2), indices)

            # print("Inliers found :" + str(len(inliers_a)) + "/" +
            #       str(len(pts1)))

            for k in indices:
                if (k not in inlier_index):
                    M[k, i - 1] = 0
                    outlier_indices[k, i - 1] = 1
                    outlier_indices[k, j - 1] = 1

    return M, outlier_indices
