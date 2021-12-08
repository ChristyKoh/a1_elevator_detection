"""
Authors(s): Christy Koh
Utility functions to support pointcloud projections etc.

Some parts adapted from EECS106A Fall 2021, Lab 6.
https://github.com/ucb-ee106/lab6_starter/blob/main/segmentation/src/pointcloud_segmentation.py
"""

import cv2
import heapq
import numpy as np
import math

def get_vertical_edges(image):
        """
        Extract vertical lines from Hough Line Transform algorithm
        """
        # tracks whether each line has been seen
        x_flags = np.zeros((640)) 
        x_flags[639] = 1

        # extract Canny edges
        dst = cv2.Canny(image, 50, 200, None, 3)

        # apply Hough Line Transform
        deg_res = math.pi/180
        vote_threshold = 150
        right_lines = cv2.HoughLines(dst, 1, deg_res, vote_threshold, None, 0, 0, deg_res)
        left_lines = cv2.HoughLines(dst, 1, deg_res, vote_threshold, None, 0, math.pi - deg_res)

        # combine line lists
        if right_lines is None:
            lines = left_lines
        elif left_lines is None:
            lines = right_lines
        else:
            lines = np.vstack((right_lines, left_lines))

        # if no lines, we still want to check the depth
        if lines is None:
            print('NO LINES')
            return [640]

        print("\n NEW LINES ")
        print(len(lines))

        # convert lines to x-values and eliminate duplicates
        for line in lines:
            rho, theta = line[0][0], line[0][1]
            # estimate by taking x-intercept
            x = int(rho * math.cos(theta))
            # print(rho, theta, x)

            if x > 0 and x_flags[x] == 0:
                x_flags[x] = 1

        # get sorted list of lines
        vertical_edges = sorted(x_flags.nonzero()[0])
        print(vertical_edges)

        # group lines if significantly close together
        delta_x = 8
        last_x = vertical_edges[0] # make sure first element is always added
        x_cluster = []
        result = []
        for x in vertical_edges:
            if x - last_x < delta_x:
                x_cluster.append(x)
            else: 
                result.append(int(np.mean(x_cluster)))
                x_cluster = [x]
            last_x = x
            
        # print(result)

        return result


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