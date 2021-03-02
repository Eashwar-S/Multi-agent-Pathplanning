import numpy as np
from Environment import *


def pointInValidWorkspace(point, res, radiusClearance):
    x, y = point

    # --------------------------------------------------------------------------------
    #                     Checking whether point inside obstacles
    # --------------------------------------------------------------------------------
    X = np.float32([8, 12.5, 12.5, 8]) / res
    Y = np.float32([9, 9, 9.5, 9.5]) / res
    ptInRectangle = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                    0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                    0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([10, 10.5, 10.5, 10]) / res
    Y = np.float32([7, 7, 14.5, 14.5]) / res
    ptInRectangle1 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([4, 4.25, 4.25, 4]) / res
    Y = np.float32([8, 8, 10.5, 10.5]) / res
    ptInRectangle2 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([1.5, 3, 3, 1.5]) / res
    Y = np.float32([9, 9, 9.25, 9.25]) / res
    ptInRectangle3 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([16, 16.25, 16.25, 16]) / res
    Y = np.float32([8, 8, 10.5, 10.5]) / res
    ptInRectangle4 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([17, 17.5, 17.5, 17]) / res
    Y = np.float32([9, 9, 9.25, 9.25]) / res
    ptInRectangle5 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) / res
    Y = np.float32([3, 3, 3.25, 3.25]) / res
    ptInRectangle6 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([10.15, 10.35, 10.35, 10.15]) / res
    Y = np.float32([0.8, 0.8, 2.3, 2.3]) / res
    ptInRectangle7 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([9, 11.5, 11.5, 9]) / res
    Y = np.float32([15, 15, 15.25, 15.25]) / res
    ptInRectangle8 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    X = np.float32([10.15, 10.40, 10.40, 10.15]) / res
    Y = np.float32([16, 16, 17.5, 17.5]) / res
    ptInRectangle9 = Y[0] - radiusClearance / res <= y <= Y[2] + radiusClearance / res and \
                     0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / res and \
                     0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / res

    if ptInRectangle or ptInRectangle1 or ptInRectangle2 or ptInRectangle3 or ptInRectangle4 or \
            ptInRectangle5 or ptInRectangle6 or ptInRectangle7 or ptInRectangle8 or ptInRectangle9:
        return False
    return True


def main():
    print(pointInValidWorkspace((11.17, 16), 1, 0))
    environment2D((11.17, 16))


if __name__ == "__main__":
    main()
