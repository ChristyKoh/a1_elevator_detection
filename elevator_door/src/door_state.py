#!/usr/bin/env python
"""
Authors(s): Christy Koh
Functions to extract elevator door state.
"""

from collections import deque
import heapq

import rospy
import numpy as np
import cv2
import math
import tf

import ros_numpy

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from elevator_door.msg import ElevatorDoorState
from utils import *

import matplotlib.pyplot as plt


class ElevatorDoorTracker:
    """
    Tracks state of elevator door from sequence of images, 
    taking into account the last n images.

    depth images are 480 x 640px
    """
    def __init__(self, img_shape=(480,640)):

        self.img_shape = img_shape

        self.prev_frame = None
        self.vertical_edges = []
        self.door_frame = [320, 320]  # default center of screen
        self.door_opening = [320, 320]  # default center

        # reference for door depth
        self.door_depth = 0.0

        # ElevatorDoorState values, default closed
        self.state = ElevatorDoorState.CLOSED
        self.is_moving = False    

    def set_door_depth_average(self, points):
        """
        Sets door depth to average depth from a Pointcloud
        """
        self.door_depth = np.average(points['z'])
        print(self.door_depth)

    def set_door_depth(self, depth):
        """ Sets door depth manually. """
        self.door_depth = depth

    def get_state(self):
        """ Returns current ElevatorDoorState. """
        return ElevatorDoorState(self.is_moving, self.state, 
                                    self.door_frame[0], self.door_frame[1],
                                    self.door_opening[0], self.door_opening[1])

    def process_state(self, points, image, cam_matrix, trans, rot, print_state=False):
        """
        Returns ElevatorDoorState, whether door is moving
        
        sample depth between all lines
        evaluate if depth is  
            1) above a threshold --> manually tune, not general
            2) above the average depth --> susceptible to inaccuracy if the floor depth dominates
            3) above the leftmost/rightmost edge depth --> 
            4) (when inside) above an initialized depth --> since first known state is closed

        use laser scan to position robot x distance away from door
        """
        depth_delta = 0.15   # min diff in depth value to be considered sig. diff
        image_delta = 8  # min diff in px distance to be sig. diff

        # using pointcloud, sample depth between each identified line
        xyz = np.vstack((points['x'], points['y'], points['z']))
        pixel_coords = project_points(xyz, cam_matrix, trans, rot)

        image_h, image_w = self.img_shape[:2]

        # mask lower half of points to ignore most of the ground
        height_lower_bound = image_h / 2
        in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                    & (height_lower_bound <= pixel_coords[1]) & (pixel_coords[1] < image_h))

        points = points[in_frame]
        pixel_coords = pixel_coords[:, in_frame]

        # calc avg depth for use in thresholding
        avg_z = np.average(points['z'])
        # if abs(avg_z - self.door_depth) < depth_delta:
        #     self.state = ElevatorDoorState.CLOSED
        #     # TODO find single line
        #     return self.get_state()

        # last depth sampled
        last_depth = 0
        # left and right bounds of image window
        left, right = 0, 0
        
        # one way flag to detect deep region
        found_deep = False
        # one way flag to detect door movement
        self.is_moving = False

        self.vertical_edges = get_vertical_edges(image, [640])
        edges = self.vertical_edges[:]

        
        # print("")
        while len(edges) > 0:
            edge = int(heapq.heappop(edges))

            left = right    # update left bound to next sample
            right = edge    # update right bound to current edge
            midpt = (right - left) // 2 + left # floordiv to bias toward left if difference is very small

            # print(left, right, midpt)
            if abs(midpt - right) < image_delta: # ignore if diff btw lines is negligible
                continue

            in_window = ((i, points[i]) for i, pt in enumerate(pixel_coords.T) 
                                        if midpt <= pt[0] < right)
            i, sample = next(in_window, (-1, []))

            if len(sample) == 0:
                print("no pointcloud data found btw x=(%d, %d)" % (left, right))
                right = left # keep the left marker
                continue

            ### determine door bounds
            sample_depth = sample[2]
            # print(sample)
            # print('L: %d   R: %d' % (left, right))
            is_deep = lambda depth: depth - depth_delta > self.door_depth
            if is_deep(sample_depth): # beyond door depth
                # DEEP!
                # print("deep %.3f %.3f" % (sample_depth, self.door_depth))
                found_deep = True
                # if print_state:
                #     print(" " * int(sample_depth) + "-")
                
                # update left door frame and opening
                if not is_deep(last_depth): # shallow -> deep
                    self.door_frame[0] = min(left, self.door_frame[0])
                    if not self.is_moving:
                        self.is_moving = abs(left - self.door_opening[0]) < image_delta
                    self.door_opening[0] = left
                
            else: # <= threshold door depth
                # SHALLOW!
                # print("shallow %.3f %.3f" % (sample_depth, self.door_depth))
                # if print_state:
                #     print("-")
                
                # update right door frame and opening
                if is_deep(last_depth): # last depth deep
                    self.door_frame[1] = max(left, self.door_frame[1])
                    if not self.is_moving:
                        self.is_moving = abs(left - self.door_opening[1]) < image_delta
                    self.door_opening[1] = left
            
            last_depth = sample_depth
        
        ### determine elevator door state
        if found_deep:
            if self.is_moving:
                # based on prev state, update state
                if self.state == ElevatorDoorState.OPEN:
                    self.state == ElevatorDoorState.CLOSING
                elif self.state == ElevatorDoorState.CLOSED:
                    self.state == ElevatorDoorState.OPENING
                # otherwise, remain in the same state
            else: # not moving
                self.state = ElevatorDoorState.OPEN
        else:
            self.state = ElevatorDoorState.CLOSED

        return self.get_state()

    def publish_annotated_image(self):
        background_img = np.zeros((480, 640, 3)).astype(np.uint8)

        # vertical edges
        for x in self.vertical_edges:
            cv2.line(background_img, (x, 0), (x, 480), (255,0,0), 3, cv2.LINE_AA)

        # elevator frame
        cv2.rectangle(background_img, (self.door_frame[0], 0), (self.door_frame[1], 480), (0,0,255), 5, cv2.LINE_AA)
        # elevator opening
        cv2.rectangle(background_img, (self.door_opening[0], 0), (self.door_opening[1], 480), (0,255,0), 3, cv2.LINE_AA)

        # publish line-annotated image
        hough_pub = rospy.Publisher('elevator/hough_lines', Image, queue_size=10)
        img_msg = ros_numpy.msgify(Image, background_img, encoding='rgb8')
        hough_pub.publish(img_msg)
