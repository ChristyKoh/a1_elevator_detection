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


def get_state(high_state):
    ''' Returns time, acceleration at current state
    '''
    _, accelerometer, _ = high_state.imu
    time = high_state.tick
    print(accelerometer)
    return [time,accelerometer]

# acceleration data

def calculate_velocity(high_state, time_passed):
    ''' Calculate if current velocity is 0 
    '''
    # get acclerometer data
    _, _, z = high_state.imu[1]
    
    # z-acceleration initally accounting for gravity
    velocity = (z - 9.8) * time_passed
    
    return velocity

# around 4/5 m: distance between floors

def calculate_distance(high_state, time_passed):
    ''' Calculate relative distance
    '''
    return calculate_velocity(high_state) * time_passed
    
    