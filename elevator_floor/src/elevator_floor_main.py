#!/usr/bin/env python
"""
Authors(s): Abigail Chin, Christy Koh

This file implements a ROS node that subscribes to /a1_states to get the imu data from the a1 unitree robot,
and publishes to elevator_floor to notify which floor the guide dog is on. 

Functions to extract elevator acceleration and height
"""
from collections import deque
from math import isnan
import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from unitree_legged_msgs.msg import HighState
from elevator_door.msg import ElevatorDoorState
from std_msgs.msg import Bool, UInt8

from elevator_floor.srv import FloorSetting
from std_srvs.srv import Empty, EmptyResponse

import altitude_calc as ac
from plotting import *


class CheckElevatorFloorProcess:
    '''
    Wraps processing of imu data from input ros topic and publishing to elevator_floor topic
    '''

    def __init__(self, a1_states, floor_pub_topic, queue_len=101):

        self.num_steps = 0

        # GRAPHING VARS
        self.accelerations_noisy = []
        self.accelerations = []
        self.time_arr = []
        self.door_states = []
        self.velocities = []
        self.displacements = []

        # CONSTANTS
        self.floor_height = 6 # TODO change estimate
        self.zero_a_threshold = 0.001

        # SIGNAL PROCESSING
        sigma = int((queue_len + 1) / 6)
        self.gaussian = ac.get_gaussian_filter(sigma, queue_len)
        self.raw_accels = deque([], queue_len)

        self.accel_offset = 9.9 # acceleration due to gravity

        # VARIABLES
        self.current_floor = 1 # TODO write service to set current floor
        self.acceleration = 9.8 # should this be 9.9?
        self.velocity = 0.0
        self.displacement = 0.0
        self.delta_t = 0.0025 # 0.002 for 500 Hz
        self.messages = []

        a1_sub = rospy.Subscriber(a1_states, HighState, self.a1_state_callback)
      
        self.elevator_floor_pub = rospy.Publisher(floor_pub_topic, UInt8, queue_size=10)

    def calibrate_offset(self, msg):
        # set acceleration to mean of raw accelerations
        self.accel_offset = np.mean(self.raw_accels)
        
        if isnan(self.accel_offset):
            self.accel_offset = 9.9
            print("failed calibration: raw_accels mean is nan")
        else:
            # reset vars
            self.velocity = 0.0
            self.displacement = 0.0
            print("calibrated offset to %.3f" % self.accel_offset)
        return EmptyResponse()

    def a1_state_callback(self, high_state):

        try:
            z_accel = high_state.imu.accelerometer[2]

            # for graphing entire sequence
            self.time_arr.append(rospy.get_time())
            self.raw_accels.append(z_accel)
            self.accelerations_noisy.append(z_accel)
            self.num_steps += 1
            
            acceleration = np.matmul(self.gaussian, self.raw_accels) - self.accel_offset

            # check if accel is 'close enough' to 0
            if abs(np.mean(self.raw_accels) - self.accel_offset) <= self.zero_a_threshold:
                # if so, reset velocity and displacement to 0 bc elevator stopped.
                acceleration = 0.0
                floor_diff = round(self.displacement / self.floor_height)
                self.current_floor += floor_diff
                print("%d floors traversed, curr floor %d." % (floor_diff, self.current_floor)) 
                # reset displacement
                print("a: %.3f  v: %.3f  d: %.3f" % (self.last_accel, self.velocity, self.displacement))
                # prevent displacement from being reset to zero when acceleration = 0 but velocity != 0
                # if abs(np.mean(self.velocities)) <= self.zero_a_threshold:
                self.displacement = 0.0
            # else push False
            
            # keep track of whether the last 1000 accelerations were basically zero.
            # if np.count_nonzero() > 5 # more than 5 accel resets, then assume elevator has stopped.
                # reset velocity and displacement to 0

            self.accelerations.append(acceleration)
            self.velocities.append(self.velocity)
            self.displacements.append(self.displacement)
            
            self.velocity += acceleration * self.delta_t

            # else, update displacement
            # TODO can create more accurate estimate by averaging past and new velocity (trapezoidal integration)
            # can have negative velocity but doesn't mean it should affect displacement
            # if 
            self.displacement += self.velocity * self.delta_t

            # print("a: %.3f  v: %.3f  d: %.3f" % (self.last_accel, self.velocity, self.displacement))

            self.elevator_floor_pub.publish(self.current_floor)

            # print('Acceleration: '+ str(acceleration))
            # self.animate(acceleration,time)
        except Exception as e:
            rospy.logerr(e)
            return

    def set_current_floor_callback(self, floor):
        # need mutex to handle multithreading?
        self.current_floor = floor
        return self.current_floor


 # def animate(i, xs, ys):
        
    #     ys = ys[-20:]
    #     xs = xs[-20:]

    #     ax.clear()
    #     ax.plot(xs, ys)

    #     plt.xticks(rotation=45, ha='right')
    #     plt.subplots_adjust(bottom=0.3)
    #     plt.title('Accelerations over Time')
    #     plt.ylabel('Acceleration (m/s^2)')

    #     ani = animation.FuncAnimation(fig, animate, fargs=(xs,ys), interval=1000)
    #     plt.show()

if __name__ == '__main__':
    IMU_TOPIC = '/a1_states'
    IMU_MSG_TYPE = 'unitree_legged_msgs/HighState'
    ELEVATOR_FLOOR_TOPIC = 'elevator_floor'

    rospy.init_node('elevator_floor', anonymous=True)
    process = CheckElevatorFloorProcess(IMU_TOPIC, ELEVATOR_FLOOR_TOPIC)
    rospy.Service('/elevator/set_current_floor', FloorSetting, process.set_current_floor_callback)
    rospy.Service('/elevator/calibrate_acceleration', Empty, process.calibrate_offset)

    r = rospy.Rate(100)
    duration = 1

    while not rospy.is_shutdown():
        # process.publish_once_from_queue()
        r.sleep()

    plot_accelerations(process)
    plot_vel_displ(process)

