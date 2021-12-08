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

    # TODO change 9.8 to calibration average
    return 9.8 - accelerometer[2]

# acceleration data

def started_moving(acceleration):
    noise_bound = 0.5
    print(acceleration)
    if acceleration <= noise_bound or acceleration >= -noise_bound:
        return False
    
    return True

def calc_vel_displacement(accelerations, times):
    ''' Calculate if current velocity is 0 
    '''
    # print(accelerations, time_arr)
    smoothed = smooth_data(accelerations)
    
    # velocity = integrate.simps(accelerations,times)
    # cum_vel = integrate.cumtrapz(accelerations,times)
    velocity = integrate.simps(accelerations, dx=.002)
    cum_vel = integrate.cumtrapz(accelerations, dx=.002)
    displacement = integrate.simps(cum_vel)

    return velocity, displacement

# around 4/5 m: distance between floors
    
''' Functions to Draw Graph
'''
def smooth_data(accelerations):
    return np.array(gaussian_filter1d(accelerations, 100))


# class ElevatorFloorState
# 