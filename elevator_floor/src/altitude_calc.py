#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This is a helper function file that takes in imu data from a 

Functions to extract elevator acceleration and height
"""
# from numpy import floor
import rospy
import numpy as np
from scipy import integrate

from unitree_legged_msgs.msg import HighState
from sensor_msgs.msg import Imu
from scipy.ndimage import gaussian_filter1d

def get_acceleration(high_state):
    ''' Returns time, acceleration at current state
    '''
    accelerometer = high_state.imu.accelerometer

    return 9.8 - accelerometer[2]

# acceleration data

def started_moving(acceleration):
    noise_bound = 0.5
    print(acceleration)
    if acceleration <= noise_bound or acceleration >= -noise_bound:
        return False
    
    return True

def calculate_velocity(accelerations, time_passed):
    ''' Calculate if current velocity is 0 
    '''
    # integrate
    # z-acceleration initally accounting for gravity
    # time_passed
    
    
    return []

# around 4/5 m: distance between floors

def calculate_distance(high_state, time_passed):
    ''' Calculate relative distance
    '''
    # how to calculate time passed with unix timestamps?
    # print('Time Passed: ' + str(time_passed))
    return calculate_velocity(high_state) * time_passed
    
''' Functions to Draw Graph
'''
def smooth_data(accelerations):
    return np.array(gaussian_filter1d(accelerations, 100))

