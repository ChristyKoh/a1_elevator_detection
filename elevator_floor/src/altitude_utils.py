#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This is a helper function file that takes in imu data from a 

Functions to extract elevator acceleration and height
"""
import rospy
import numpy as np
from math import sqrt, exp, pi
from scipy import integrate

from unitree_legged_msgs.msg import HighState
from sensor_msgs.msg import Imu
from scipy.ndimage import gaussian_filter1d


def get_gaussian_filter(sigma, n):
    r = range(-int(n/2),int(n/2)+1)
    return [1 / (sigma * sqrt(2*pi)) * exp(-float(x)**2/(2*sigma**2)) for x in r]

# TODO implement Kalman filtering


### Ignore Unused Functions below

def calc_vel_displacement(accelerations, delta_t):
    ''' Calculate velocity and displacement
    '''
    # print(accelerations, time_arr)
    smoothed = smooth_data(accelerations)
    
    # velocity = integrate.simps(accelerations,times)
    # cum_vel = integrate.cumtrapz(accelerations,times)
    velocity = integrate.simps(accelerations, dx = delta_t)
    cum_vel = integrate.cumtrapz(accelerations, dx = delta_t)
    displacement = integrate.simps(cum_vel)

    return velocity, displacement
    
def calc_displ(velocities, delta_t):
    return integrate.simps(velocities,dx =  delta_t) 

# around 4/5 m: distance between floors
    
''' Functions to Draw Graph
'''
def smooth_data(accelerations):
    return np.array(gaussian_filter1d(accelerations, 100))


