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
    def __init__(self, queue_len=5):
        self.queue_len = queue_len

        self.prev_frame = None
        self.vertical_edges = []
        self.door_frame = [0, 640]  # default entire screen
        self.door_opening = [0, 0]  # default leftmost

        # reference for door depth
        self.door_depth = 0.0

        # ElevatorDoorState values, default closed
        self.state = ElevatorDoorState.CLOSED
        self.is_moving = False    

    def set_door_depth(self, depth):
        self.door_depth = depth

    def get_state(self, points, image, cam_matrix, trans, rot, print_state=False):
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
        depth_delta = 0.1 # min diff in depth value to be considered sig. diff
        image_delta = 20  # min diff in px value to be sig. diff

        # using pointcloud, sample depth between each identified line
        xyz = np.vstack((points['x'], points['y'], points['z']))
        pixel_coords = project_points(xyz, cam_matrix, trans, rot)
        
        image_h, image_w = image.shape[:2]

        # take top points to cut out most of the ground
        height_lower_bound = image_h / 2
        in_frame = ((0 <= pixel_coords[0]) & (pixel_coords[0] < image_w)
                    & (height_lower_bound <= pixel_coords[1]) & (pixel_coords[1] < image_h))

        points = points[in_frame]
        pixel_coords = pixel_coords[:, in_frame]

        # calc avg depth for use in thresholding
        avg_z = np.average(points['z'])

        # left and right bounds of elevator
        door_left, door_right = -1, -1
        last_depth = 0

        # left and right bounds of image window
        left = 0
        right = 0
        # one way flag to detect deep region
        found_deep = False
        # one way flag to detect door movement
        self.is_moving = False

        self.vertical_edges = get_vertical_edges(image, [640])
        edges = self.vertical_edges
        # self.vertical_edges, canny = get_vertical_edges(image, [640])
        # publish_annotated_image(self.vertical_edges, canny)

        while len(edges) > 0:
            edge = int(heapq.heappop(edges))

            left = right    # update left bound to next sample
            right = edge    # update right bound to current edge
            midpt = (right - left) // 2 + left # floordiv to bias toward left if difference is very small

            # print(right, left, midpt)
            if abs(midpt) < 10: # ignore if diff btw lines is negligible
                continue

            # sample first depth point
            in_window = ((i, points[i]) for i, pt in enumerate(pixel_coords.T) 
                                            if midpt <= pt[0] < right)
            i, sample = next(in_window, (-1, []))
            
            # print(sample)
            # print(pixel_coords[:,i])

            if len(sample) == 0:
                print("no pointcloud data found between x-vals %d and %d" % (left, right))
                continue

            ### determine door bounds
            sample_depth = sample[2]

            is_deep = lambda depth: depth - depth_delta > self.door_depth
            if is_deep(sample_depth): # beyond door depth
                # DEEP!
                # print("deep")
                found_deep = True
                if print_state:
                    print(" " * int(sample_depth) + "-")
                
                # update left door frame and opening
                if not is_deep(last_depth): # shallow -> deep
                    self.is_moving = True
                    self.door_frame[0] = min(left, self.door_frame[0])
                    self.door_opening[0] = left
                
            else: # <= threshold door depth
                # SHALLOW!
                # print("shallow")
                if print_state:
                    print("-")
                
                # update right door frame and opening
                if is_deep(last_depth): # last depth deep
                    self.is_moving = True
                    self.door_frame[1] = max(right, self.door_frame[1])
                    self.door_opening[1] = right

            ### determine elevator door state
            if found_deep:
                if self.is_moving:
                    # based on prev state, update state
                    if self.state == ElevatorDoorState.OPEN:
                        self.state == ElevatorDoorState.CLOSING
                    elif self.state == ElevatorDoorState.CLOSED:
                        self.state == ElevatorDoorState.OPENING
                else: # not moving
                    self.state = ElevatorDoorState.OPEN
            else:
                self.state = ElevatorDoorState.CLOSED
            
            last_depth = sample_depth

        door_state = ElevatorDoorState(self.is_moving, self.state, 
                                        self.door_frame[0], self.door_frame[1],
                                        self.door_opening[0], self.door_opening[1])
        if print_state:
            print(door_state)
            print('')

        return door_state

    def publish_annotated_image(self):
        background_img = np.zeros((480, 640, 3)).astype(np.uint8)

        # vertical edges
        for x in self.vertical_edges:
            cv2.line(background_img, (x, 0), (x, 480), (255,0,0), 3, cv2.LINE_AA)

        # elevator frame
        cv2.line(background_img, (self.door_frame[0], 0), (self.door_frame[0], 480), (0,0,255), 3, cv2.LINE_AA)
        cv2.line(background_img, (self.door_frame[1], 0), (self.door_frame[1], 480), (0,0,255), 3, cv2.LINE_AA)

        # elevator opening
        cv2.line(background_img, (self.door_opening[0], 0), (self.door_opening[0], 480), (0,255,0), 3, cv2.LINE_AA)
        cv2.line(background_img, (self.door_opening[1], 0), (self.door_opening[1], 480), (0,255,0), 3, cv2.LINE_AA)

        # publish line-annotated image
        hough_pub = rospy.Publisher('elevator/hough_lines', Image, queue_size=10)
        img_msg = ros_numpy.msgify(Image, background_img, encoding='rgb8')
        hough_pub.publish(img_msg)



def determine_elevator_state(image):
    """
    Based on image/depth data, return the state and
    image with the elevator door boundaries annotated.
    """

    # edge_image = cv2.Canny(image, 200, 250)

    return edge_image, ElevatorDoorState(False, ElevatorDoorState.CLOSED)