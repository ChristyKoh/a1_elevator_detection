#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This is a helper function file that takes in imu data from a 

Functions to extract elevator acceleration and height
"""
from numpy import floor
import rospy

from unitree_legged_msgs import HighState
from sensor_msgs.msg import IMU

def get_altitude(high_state):
    _, accelerometer, _ = high_state.imu
    print(accelerometer)
    return 0.0

def calculate_floor(accelerometer_arr):
    
    return 0.0

