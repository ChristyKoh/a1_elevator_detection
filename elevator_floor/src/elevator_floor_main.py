#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This file implements a ROS node that subscribes to /a1_states to get the imu data from the a1 unitree robot,
and publishes to elevator_floor to notify which floor the guide dog is on. 

Functions to extract elevator acceleration and height
"""
from collections import deque

import message_filters
import rospy
import datetime

from unitree_legged_msgs.msg import HighState
from elevator_door.msg import ElevatorDoorState
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray

import altitude_calc as ac


class CheckElevatorFloorProcess:
    '''
    Wraps processing of imu data from input ros topic and publishing to elevator_floor topic
    '''

    def __init__(self, a1_states, floor_node, elevator_door, diagnostic_node):
        self.num_steps = 0
        self.time_arr = []
        self.is_moving = False
        self.forward_velocity = 0.0
        self.accelerations = []
        self.door_states = []
        
        self.messages = deque([], 5)
        a1_sub = rospy.Subscriber(a1_states, HighState, self.a1_state_callback)
        diagnostic_sub = rospy.Subscriber(diagnostic_node, DiagnosticArray, self.time_callback)
        door_sub = rospy.Subscriber(elevator_door, ElevatorDoorState, self.elevator_state_callback)

        self.elevator_floor_pub = rospy.Publisher(
            floor_node, Bool, queue_size=10)

    def time_callback(self, diagnostic):
        try:
            time = diagnostic.header.stamp
            self.time_arr.append(time)
            # print('Time:'+ str(diagnostic.header.stamp))
        except Exception as e:
            rospy.logerr(e)
            return

    def a1_state_callback(self, high_state):
        try:
            acceleration = ac.get_acceleration(high_state)
            # print('Acceleration: '+ str(acceleration))
            self.is_moving =  acceleration > 1 
            print(self.is_moving)
        except Exception as e:
            rospy.logerr(e)
            return
        
        self.messages.appendleft(high_state)

    def elevator_state_callback(self, door_state):
        try:
            # why is door state not showing? 
            # compare past door state
            if door_state is ElevatorDoorState.CLOSED:
                print('closed')
            elif door_state is ElevatorDoorState.OPEN:
                print('open')
            # print('Door State:' + str(door_state))
            past_door_state = ElevatorDoorState.CLOSED
            if len(door_state > 1):
                past_door_state = self.door_states.pop(0)
            
            # if not moving forward, should be in elevator
            # is_not_moving =  high_state.velocity[0] == 0 
            if self.is_not_moving:
                # self.start_time = # set start time here? or end time?
                if door_state is ElevatorDoorState.OPENING and past_door_state is ElevatorDoorState.CLOSED:
                    print('opening')
                # check if starting to move
                elif door_state is ElevatorDoorState.CLOSING:  
                    print('closing')
                    # self.start_time = # get time here
                    # want the time between not moving states to calculate distance
                    # a1_state = self.get_altitude(high_state)
            self.door_states.append(door_state)
        
        except Exception as e:
            rospy.logerr(e)
            return
        
        # self.messages.appendleft([high_state, diag_sub, door_state])

    def publish_once_from_queue(self):
        if self.messages:
            # let's just publish after after not_moving->not_moving time period
            high_state = self.messages.pop()
            # acceleration = high_state.imu.accelerometer[2]
            try:
                time_passed = self.time_arr[-1] - self.time_arr[0]
                distance = ac.calculate_distance(high_state, time_passed)
                # print('Distance: ' + str(distance))
            except Exception as e:
                return

            self.elevator_floor_pub.publish(False)
            # print("Published floor msg at timestamp:",
            #       floor.header.stamp.secs)


def main():
    IMU_NODE = '/a1_states'
    IMU_MSG_TYPE = 'unitree_legged_msgs/HighState'
    ELEVATOR_FLOOR_NODE = 'elevator_floor'
    ELEVATOR_DOOR_NODE = 'elevator_door'
    DIAGNOSTIC_NODE = '/diagnostics'

    rospy.init_node('elevator_floor', anonymous=True)
    process = CheckElevatorFloorProcess(IMU_NODE, ELEVATOR_FLOOR_NODE, ELEVATOR_DOOR_NODE, DIAGNOSTIC_NODE)

    r = rospy.Rate(1000)

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()


if __name__ == '__main__':
    main()
