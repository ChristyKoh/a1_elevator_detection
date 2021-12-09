#!/usr/bin/env python
"""
Authors(s): Christy Koh
Controller node to handle elevator entry/exit pipeline.
"""

import rospy
from elevator_door.msg import ElevatorDoorState
from std_msgs.msg import Uint8


class ElevatorControl:
    
    def __init__(self, floor):
        self.door_sub = rospy.Subscriber('/elevator/door_state', ElevatorDoorState)
        self.floor_sub = rospy.Subscriber('/elevator/floor', ElevatorDoorState)

        self.state_pub = rospy.Publisher('/elevator/ctrl/state', Uint8)

        # State variables
        self.current_floor = floor
        self.target_floor = floor
        self.state = 1
        # state 1: outside elevator, entering
            # wait till elevatorDoorState open
            # walk till w/in ~ dist from back wall
        # state 2: inside elevator, facing wall
            # turn 180deg
        # state 3: inside elevator, facing door
            # wait till elevatorDoorState open
            # wait till floor is target_floor
            # repeat
        # state 4: inside elevator
            # wait till elevatorDoorState open
            # walk outside for set amount of time
        # state 5: outside elevator, exited
            # wait for global/local planner

    def set_target(self, floor):
        self.target_floor = floor

    def set_current(self, floor):
        self.current_floor = floor

    def publish_state(self):
        self.state_pub.publish(self.state)


if __name__ == '__main__':
    rospy.init_node('elevator_control')

    control = ElevatorControl(2)

    # rospy.Timer()