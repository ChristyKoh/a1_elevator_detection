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
from elevator_door.msg import ElevatorDoorState
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

import altitude_calc as ac


class CheckElevatorFloorProcess:
    '''
    Wraps processing of imu data from input ros topic and publishing to elevator_floor topic
    '''

    def __init__(self, a1_states, floor_node, elevator_door):
        self.num_steps = 0
        self.start_time = 0.0
        self.end_time = 0.0
        self.is_moving_in_z_dir = False
        self.accelerations = []
        self.door_states = []
        
        self.messages = deque([], 5)
        a1_sub = rospy.Subscriber(a1_states, HighState)
        door_sub = rospy.Subscriber(elevator_door, ElevatorDoorState)

        self.elevator_floor_pub = rospy.Publisher(
            floor_node, Bool, queue_size=10)
        self.callback(a1_sub, door_sub)

    def callback(self, high_state, door_state):
        try:
            # compare past door state
            past_door_state = ElevatorDoorState.CLOSED
            if len(door_state > 1):
                past_door_state = self.door_states.pop(0)
            
            # if not moving forward, should be in elevator
            # is_not_moving =  high_state.velocity[0] == 0 
            is_not_moving = ac.calculate_velocity(high_state) <= 1.0
            if is_not_moving:
                self.start_time = # set start time here? or end time?
                if door_state is ElevatorDoorState.OPENING and past_door_state is ElevatorDoorState.CLOSED:
                    print('opening')
                # check if starting to move
                elif door_state is ElevatorDoorState.CLOSING:  
                    print('closing')
                    self.start_time = # get time here
                    # want the time between not moving states to calculate distance
                    a1_state = self.get_altitude(high_state)
            self.door_states.append(door_state)
        
        except Exception as e:
            rospy.logerr(e)
            return
        
        self.messages.appendleft(a1_state)

    def publish_once_from_queue(self):
        if self.messages:
            # let's just publish after after not_moving->not_moving time period
            time, acceleration = self.messages.pop()
            try:
                floor = ac.calculate_distance(acceleration)
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
    process = CheckElevatorFloorProcess(IMU_NODE, ELEVATOR_FLOOR_NODE)

    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
