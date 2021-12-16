#!/usr/bin/env python
"""
Authors(s): Christy Koh
Controller node to handle elevator entry/exit pipeline.
"""
import time

import rospy
from elevator_door.msg import ElevatorDoorState
from std_msgs.msg import UInt8


class ElevatorControl:
    
    def __init__(self, floor):
        self.floor_sub = rospy.Subscriber('/elevator/floor', UInt8, self.set_current)
        self.door_sub = rospy.Subscriber('/elevator/door_state', ElevatorDoorState, self.update_door_state)

        self.state_pub = rospy.Publisher('/elevator/ctrl/state', UInt8, queue_size=10)


        # State variables
        self.current_floor = floor
        self.target_floor = 1
        self.door_state = ElevatorDoorState(False, ElevatorDoorState.CLOSED, 0, 0, 0, 0)

        self.control_state = 1
        print("\n---------------------------------------")
        print("State 1: Wait for elevator door to open")
        print("---------------------------------------")

        print("DOOR CLOSED")
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

    def update_door_state(self, door_state):
        self.door_state = door_state

    def set_target(self, floor):
        self.target_floor = floor

    def set_current(self, floor):
        self.current_floor = floor

    def publish_state(self):
        self.state_pub.publish(self.control_state)

    def state_controller(self):
        if self.control_state == 1:
            if self.door_state.state == ElevatorDoorState.OPEN:
                print("DOOR OPEN")
                self.control_state = 2
                # TODO call CPP service to walk forward and turn 180deg
        elif self.control_state == 2:
            print("\n---------------------------------------")
            print("State 2: Turn and face elevator door")
            print("---------------------------------------")
            # TODO wait till robot is finished moving
            time.sleep(5)

            print("DOOR CLOSED")
            self.control_state = 3

            print("\n---------------------------------------")
            print("State 3: Wait to arrive at target floor")
            print("---------------------------------------")
            print("current floor: 2")
        elif self.control_state == 3:
            # if self.current_floor == self.target_floor:
            time.sleep(9)
            print("arrived at floor: 1")
            self.control_state = 4
            print("\n---------------------------------------")
            print("State 4: Exit elevator when open")
            print("---------------------------------------")
        elif self.control_state == 4:
            if self.door_state == ElevatorDoorState.OPEN:
                print("DOOR OPEN")
                time.sleep(4)
                self.control_state = 5
                # TODO call CPP service to exit elevator
        elif self.control_state == 5:
            print("\n---------------------------------------")
            print("State 5: Complete")
            print("---------------------------------------")

        self.publish_state()


if __name__ == '__main__':
    rospy.init_node('elevator_control')

    control = ElevatorControl(2)

    r = rospy.Rate(5)

    while not rospy.is_shutdown():
        control.state_controller()
        r.sleep()
