#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This is a helper function file that takes in imu data from a 

Functions to extract elevator acceleration and height
"""
from numpy import floor
import rospy

from unitree_legged_msgs.msg import HighState
from sensor_msgs.msg import Imu



def calculate_floor(accelerometer_arr):
    
    return 0.0

