#!/usr/bin/env python
"""
Authors(s): Christy Koh
Functions to extract elevator door state.
"""

from collections import deque

import rospy
import numpy as np
import cv2
import math

import ros_numpy

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from elevator_door.msg import ElevatorDoorState

import matplotlib.pyplot as plt


class ElevatorStateTracker:
    """
    Tracks state of elevator door from sequence of images, 
    taking into account the last n images.

    depth images are 480 x 640px
    """
    def __init__(self, queue_len=5):
        self.queue_len = queue_len
        self.vertical_edges = deque([], queue_len)

        # reference for door depth
        self.door_depth = 0

        # ElevatorDoorState values, default closed
        self.state = ElevatorDoorState.CLOSED
        self.is_moving = False
        self.is_inside_view = False

    def is_vertical(self, line):
        """
        Returns True if given a line returned by Hough line detector, its slope 
        theta (angle from the x-axis) is within delta of pi/2.
        """
        theta = line[0][1]
        print(theta)
        delta = 0.1
        return min(abs(theta - math.pi), theta) <= delta

    def apply_hough_line_tfm(self, image):

        # PERFORM HOUGH LINE TRANSFORM
        src = image

        dst = cv2.Canny(src, 50, 200, None, 3)
    
        # Copy edges to the images that will display the results in BGR
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        
        lines = cv2.HoughLines(dst, 1, np.pi / 180, 100, None, 0, 0)

        if lines is not None:

            # print(lines)
            # print("hough detector num lines is %d" % len(lines))

            # filter for only near-vertical lines
            vert_lines = filter(self.is_vertical, lines)
            print("vertical filtered num lines is %d" % len(vert_lines))

            lines = vert_lines
            self.vertical_edges = sorted(vert_lines, key=lambda x: x[0][0])
            print(self.vertical_edges)

            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)  
            
            # draw ref line
            cv2.line(cdst, (0,0), (0, 300), (0,0,255), 3, cv2.LINE_AA)

        # publish identified lines
        hough_pub = rospy.Publisher('elevator/hough_lines', Image, queue_size=10)
        img_msg = ros_numpy.msgify(Image, cdst, encoding='rgb8')
        hough_pub.publish(img_msg)

    def get_state(self, points, image, info):
        """
        Returns ElevatorDoorState, whether door is moving
        
        sample depth between all lines
        evaluate if depth is  
            1) above a threshold --> manually tune, not general
            2) above the average depth --> susceptible to inaccuracy if the floor depth dominates
            3) above the leftmost/rightmost edge depth --> 
            4) (when inside) above an initialized depth --> since first known state is closed

        use laser scan to position robot x distance away from door

        note: sometimes have blackouts
        """
        delta = 100 # min diff in depth img value to be considered "deep"

        # using pointcloud, sample depth between each identified line
        xz = np.vstack((points['x'], points['z']))
        # sort by x
        xz = sorted(xz, key=lambda pt: pt[0])
        avg_z = np.mean(xz, axis=0)

        # using depth image
        # image_avg = np.mean(image, axis=0) # take mean depth across y-values
        # image_grad = np.gradient(image_avg)

        if len(self.vertical_edges):

            # left and right bounds of elevator door (relatively increased depth)
            left = 0.0
            right = 0.0
            # one way flag to find transition from deep -> shallow
            found_deep = False

            for edge in self.vertical_edges:
                left = right          # update left edge so next sample
                right = edge[0][0]    # take rho
                midpt = (right - left) // 2 + left # floordiv to bias toward left if difference is very small

                print(right, left, midpt)
                if abs(midpt) < 10: # ignore if diff btw lines is negligible
                    continue

                i, sample = next(((i, pt) for (i, pt) in enumerate(xz) if midpt <= pt[0] < right), (-1, []))

                print(i, sample)
                if len(sample) == 0:
                    print("none found")
                elif sample - delta > self.door_depth:
                    # deeper than door depth
                    print("deep")
                else:
                    found_deep = True
                    print("shallow")

        
        if self.state == ElevatorDoorState.CLOSED: # update door distance when closed
            # using pointcloud
            self.door_depth = (self.door_depth + avg_z) / 2
            # using depth image
            # self.door_depth = (self.door_depth + image_avg[400]) // 2
            print("door depth", self.door_depth)


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

        return ElevatorDoorState(False, ElevatorDoorState.CLOSED)

def determine_elevator_state(image):
    """
    Based on image/depth data, return the state and
    image with the elevator door boundaries annotated.
    """

    # edge_image = cv2.Canny(image, 200, 250)

    return edge_image, ElevatorDoorState(False, ElevatorDoorState.CLOSED)