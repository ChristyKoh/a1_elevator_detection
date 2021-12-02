"""
Authors(s): Christy Koh
Utility functions to support pointcloud projections etc.

Adapted from https://github.com/ucb-ee106/lab6_starter/blob/main/segmentation/src/pointcloud_segmentation.py
"""

import cv2
import heapq
import numpy as np
import math

def is_vertical(line):
    """
    Returns True if given a line returned by Hough line detector, its slope 
    theta (angle from the x-axis) is within delta of pi/2.
    """
    delta = 0.1

    theta = line[0][1]
    return min(abs(theta - math.pi), theta) <= delta

def get_vertical_edges(image, init=[]):
        """
        Extract vertical lines from Hough Line Transform algorithm
        """
        delta = 0.1

        vertical_edges = init  # init with parameter
        src = image

        deg_res = math.pi/180

        # extract Canny edges
        dst = cv2.Canny(src, 50, 200, None, 3)
        # apply Hough Line Transform
        right_lines = cv2.HoughLines(dst, 1, deg_res, 100, None, 0, 0, deg_res)
        left_lines = cv2.HoughLines(dst, 1, deg_res, 100, None, 0, math.pi - 0.15)
        # right_lines = None

        if right_lines is None:
            lines = left_lines
        elif left_lines is None:
            lines = right_lines
        else:
            lines = np.vstack((right_lines, left_lines))

        # lines = cv2.HoughLines(dst, 1, deg_res, 150, None, 0, 0)


        if lines is None:
            print('\nNO LINES\n')
            return []


        print(len(lines))
        # filter for only near-vertical lines
        for line in lines:
            # if not is_vertical(line):
            #     continue

            rho, theta = line[0][0], line[0][1]
            x = int(rho * math.cos(theta))

            heapq.heappush(vertical_edges, x)

        return vertical_edges


def project_points(pts, cam_matrix, trans, rot):
    """ Projects pointcloud onto RGB image plane.
    """

    points = np.dot(rot, pts) + trans[:, None]
    homo_pixel_coords = np.dot(cam_matrix, points)
    pixel_coords = homo_pixel_coords[:2] / homo_pixel_coords[2]
    pixel_coords = np.floor(pixel_coords).astype(np.int32)

    return pixel_coords



        # if len(self.vertical_edges) == n:
            # TODO calculate gradient between subsequent images
            # if is_moving --> closing or opening
            # once stopped, closing --> closed and opening --> open
        # else:
            # TODO add to vertical edges
        # plt.imshow(image)

        # plt.title("x pos and depth")
        # plt.scatter(points['x'], points['z'])
        # plt.show()

        #plt.title("elevator depth histogram along x")
        #image_avg = np.mean(image, axis=0) # take mean depth across y-values
        # image_avg = np.max(image, axis=0) # take max depth across y-values
        #image_grad = np.gradient(image_avg)
        # print(image.shape)
        # print(len(image_avg))
        #plt.scatter(np.arange(image.shape[1]), image_grad)
        # plt.hist(image)
        # plt.hist(points['z'])
        #plt.show()

        # avg_z = np.mean(xz, axis=0)
        # avg_depth_pub = rospy.Publisher('elevator/avg_depth', Float32, queue_size=10)
        # avg_depth_pub.publish(avg_z)