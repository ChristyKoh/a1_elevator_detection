#!/usr/bin/env python
"""
Authors(s): Christy Koh
Functions to extract elevator door state.
"""

from collections import deque

import numpy as np
import cv2

from sensor_msgs.msg import Image
from elevator_door.msg import ElevatorDoorState

import matplotlib.pyplot as plt

class ElevatorStateTracker:
    """
    Tracks state of elevator door from sequence of images, 
    taking into account the last n images.
    """
    def __init__(self, n):
        self.n = n
        self.vertical_edges = deque([], n)

    def get_state(image):
        # if len(self.vertical_edges) == n:
            # TODO calculate gradient between subsequent images
            # if is_moving --> closing or opening
            # once stopped, closing --> closed and opening --> open
        # else:
            # TODO add to vertical edges
        return ElevatorDoorState.CLOSED

def determine_elevator_state(image):
    """
    Based on image/depth data, return the state and
    image with the elevator door boundaries annotated.
    """

    edge_image = cv2.Canny(image, 200, 250)

    return edge_image, ElevatorDoorState(False, ElevatorDoorState.CLOSED)