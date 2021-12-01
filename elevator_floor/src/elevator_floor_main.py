#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This file implements a ROS node that subscribes to /a1_states to get the imu data from the a1 unitree robot,
and publishes to elevator_floor to notify which floor the guide dog is on. 

Functions to extract elevator acceleration and height
"""
import numpy as np
from collections import deque

import rospy

from unitree_legged_msgs.msg import HighState
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

import altitude_calc as ac

class CheckElevatorFloorProcess:
    '''
    Wraps processing of imu data from input ros topic and publishing to elevator_floor topic
    '''
    def __init__(self, a1_states, floor_node):
        self.num_steps = 0
        self.start_time = 0.0
        self.is_moving_in_z_dir = False
        
        self.messages = deque([], 5)
        a1_sub = rospy.Subscriber(a1_states, HighState, self.callback)
        
        self.elevator_floor_pub = rospy.Publisher(floor_node, Bool, queue_size=10)
        # self.callback(a1_sub)
    
    def callback(self, high_state):
        try:
            a1_state = self.get_altitude(high_state)
        except Exception as e:
            rospy.logerr(e)
            return
        self.messages.appendleft(a1_state)
    
    def get_altitude(self, high_state):
        x,y,z_acceleration = high_state.imu.accelerometer
        print('position:' + str(high_state.position))
        print('z_acceleration:'+ str(z_acceleration))
        # initially around 9.8 m/s^2
        # calculate when moving and then 
            # calculate previous iteration of moving 
        # else:
        #    self.is_moving_in_z_dir = True
        return 0.0
        
    def publish_once_from_queue(self):
        if self.messages:
            a1_state = self.messages.pop()
            try:
                floor = ac.calculate_floor(a1_state)
            except Exception as e:
                return
             
            self.elevator_floor_pub.publish(False)
            # print("Published floor msg at timestamp:",
            #       floor.header.stamp.secs)

def main():
    IMU_NODE = '/a1_states'
    IMU_MSG_TYPE = 'unitree_legged_msgs/HighState'
    ELEVATOR_FLOOR_NODE = 'elevator_floor'


    rospy.init_node('elevator_floor', anonymous=True)
    process = CheckElevatorFloorProcess(IMU_NODE,ELEVATOR_FLOOR_NODE)
    
    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()

if __name__ == '__main__':
    main()
    
    
