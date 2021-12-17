#!/usr/bin/env python
"""
Authors(s): Christy Koh
Controller node to handle elevator entry/exit pipeline.
"""
import time

import rospy
from a1_control_states import *

from elevator_door.msg import ElevatorDoorState
from std_msgs.msg import UInt8, Float64
from unitree_legged_msgs.msg import HighCmd, HighState
from std_srvs.srv import Empty

DEPTH_THRESHOLD = 600  # in pointcloud units
UPDATE_DELAY = 0.002  # Hz

class ElevatorControl:
    
    def __init__(self, floor, ctrl_state):
        self.floor_sub = rospy.Subscriber('/elevator/floor', UInt8, self.set_current_floor)
        self.door_sub = rospy.Subscriber('/elevator/door_state', ElevatorDoorState, self.update_door_state)
        self.a1_state_sub = rospy.Subscriber('/a1_states', HighState, self.update_a1_state)
        self.depth_sub = rospy.Subscriber('/elevator/avg_depth', Float64, self.update_depth)

        self.state_pub = rospy.Publisher('/elevator/ctrl/state', UInt8, queue_size=1)
        self.cmd_pub = rospy.Publisher('/elevator/ctrl/highcmd', HighCmd, queue_size=10)

        self.calibrate_depth = rospy.ServiceProxy('/calibrate_depth', Empty)

        # State variables
        self.current_floor = floor
        self.target_floor = 1
        self.view_depth = 0.0  # avg depth of Realsense PointCloud
        self.door_state = ElevatorDoorState()

        self.yaw = 0.0

        self.control_state = ctrl_state
        print_state(self.control_state)

        self.is_complete = False

    # -------- INSTANCE SETTER METHODS --------

    def update_door_state(self, door_state):
        self.door_state = door_state

    def update_a1_state(self, high_state):
        self.yaw += high_state.imu.gyroscope[2] * UPDATE_DELAY
        # print("gyro: \t%.3f \t%.3f \t%.3f" % self.high_state.imu.gyroscope)

    def update_depth(self, depth):
        self.view_depth = depth
        print("depth: %.3f" % self.view_depth)

    def set_target_floor(self, floor):
        self.target_floor = floor

    def set_current_floor(self, floor):
        self.current_floor = floor

    # ------- PUBLISHING METHODS --------

    def publish_state(self):
        self.state_pub.publish(self.control_state)

    def command_stop(self):
        cmd = HighCmd()
        cmd.mode = 1
        self.cmd_pub.publish(cmd)

    def command_walk(self):
        cmd = HighCmd()
        cmd.mode = 2
        self.cmd_pub.publish(cmd)

    def command_rotate(self, rot_speed):
        cmd = HighCmd()
        cmd.mode = 2
        cmd.rotateSpeed = rot_speed
        self.cmd_pub.publish(cmd)

    def command_side(self, side_speed):
        cmd = HighCmd()
        cmd.mode = 2
        cmd.sideSpeed = side_speed
        self.cmd_pub.publish(cmd)

    def command_fwd(self, fwd_speed):
        cmd = HighCmd()
        cmd.mode = 2
        cmd.forwardSpeed = fwd_speed
        self.cmd_pub.publish(cmd)

    def publish_highcmd(self, cmd):
        self.cmd_pub.publish(cmd)

    # -------- STATE MACHINE LOOP --------

    def state_controller(self):
        if self.control_state == ControlState.OUTSIDE_WAITING:
            # wait until door open
            if self.door_state.state == ElevatorDoorState.OPEN:
                print("DOOR OPEN")
                # transition: start entering
                self.command_fwd(0.2)
                self.control_state = self.control_state.next()
        elif self.control_state == ControlState.ENTERING:
            # walk until pointcloud estimate
            if self.depth_sub < DEPTH_THRESHOLD:
                # transition: stop moving
                self.command_stop()
                # TODO initiate button press
                self.control_state = self.control_state.next()
        elif self.control_state == ControlState.PRESSING_BUTTON:
            print("target floor: ", self.target_floor)
            # transition: rotate to face door
            self.yaw = 0.0
            self.command_rotate(2.4)
            self.control_state = self.control_state.next()
        elif self.control_state == ControlState.TURNING_TO_DOOR:
            print("yaw: %.3f" % self.yaw)
            if self.yaw > 3.14:
                print("TURNED 180 DEGREES.")
                # TODO Check if robot has rotated 180 degrees
                # transition: enter joint locked mode and calibrate depth
                self.command_stop()
                time.sleep(1)
                self.calibrate_depth()
                time.sleep(1)
                self.control_state = self.control_state.next()
        elif self.control_state == ControlState.INSIDE_WAITING:
            if self.current_floor == self.target_floor:
                print("arrived at floor: ", self.current_floor)
                # transition: start exiting
                self.command_fwd(0.2)
                self.control_state = self.control_state.next()
        elif self.control_state == ControlState.EXITING:
            if self.door_state == ElevatorDoorState.OPEN:
                print("DOOR OPEN")
                # transition: stop movement
                self.command_stop()
                self.control_state = self.control_state.next()
        elif self.control_state == ControlState.COMPLETE:
            self.is_complete = True

        self.publish_state()


if __name__ == '__main__':
    rospy.init_node('elevator_control')
    init_state = rospy.get_param('~init_state', 1)
    rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~init_state'), init_state)

    control = ElevatorControl(2, ControlState(init_state))

    control.command_walk()
    time.sleep(0.5)
    control.command_fwd(0.3)
    time.sleep(2)
    control.command_stop()

    print("complete")

    r = rospy.Rate(10)

    while not rospy.is_shutdown() and not control.is_complete:
        control.state_controller()
        r.sleep()
