#!/usr/bin/env python
"""
Authors(s): Abigail Chin

This file implements a ROS node that subscribes to /a1_states to get the imu data from the a1 unitree robot,
and publishes to elevator_floor to notify which floor the guide dog is on. 

Functions to extract elevator acceleration and height
"""
from collections import deque
import numpy as np
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy import integrate

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
        self.is_moving = False
        self.forward_velocity = 0.0
        self.accelerations = []
        self.time_arr = []
        self.door_states = []

        # CONSTANTS
        self.floor_height = 6 # TODO change estimate

        # VARIABLES
        self.current_floor = 1 # TODO write service to set current floor
        self.past_accels = deque([], 100)
        self.velocity = 0.0
        self.displacement = 0.0
        self.delta_t = 0.002
        
        # messages as deque (with max length = 5) instead of list?
        self.messages = []
        a1_sub = rospy.Subscriber(a1_states, HighState, self.a1_state_callback)
      
        self.elevator_floor_pub = rospy.Publisher(
            floor_node, Bool, queue_size=10)

    def a1_state_callback(self, high_state):
        try:
            acceleration = ac.get_acceleration(high_state)
            # is the time the rospy time of the bag file?
            # time =  rospy.Time.now() 

            # for graphing entire sequence
            self.time_arr.append(rospy.get_time())
            self.accelerations.append(acceleration)
            self.num_steps += 1

            # TODO apply gaussian 
            self.velocity += acceleration * self.delta_t

            # check if velocity is 'close enough' to 0
            # if so, reset velocity and displacement to 0 bc elevator stopped.
                # calculate - round(curr_displ / floor_height) == number of floors traversed

            # else, update displacement
            # TODO can create more accurate estimate by averaging past and new velocity (trapezoidal integration)
            self.displacement += self.velocity * self.delta_t

            self.last_accel = acceleration

            print("Velocity: %.3f" % self.velocity)
            print("Displacement: %.3f" %self.displacement)

            # TODO publish 

            # print('Acceleration: '+ str(acceleration))
            # self.animate(acceleration,time)
        except Exception as e:
            rospy.logerr(e)
            return
        
        self.messages.append([acceleration, rospy.get_time()])

    def publish_once_from_queue(self, event):
        
        if self.messages:
            # let's just publish after after not_moving->not_moving time period
            accelerations = []
            times = []
            for each in self.messages:
                accelerations.append(each[0])
                times.append(each[1])
            
            self.messages = []
            
            # sduration = (time - self.time_arr[-1]).toSec()
            try:
                # if not self.is_moving:
                # 100 Hz / sample len(a) = time (s)
                vel,dist = ac.calc_vel_displacement(accelerations, times)
                print('Velocity: ' + str(vel))
                print('Distance: ' + str(dist))
            except Exception as e:
                print(e)
                return

            self.elevator_floor_pub.publish(False)
            # print("Published floor msg at timestamp:",
            #       floor.header.stamp.secs)

def plot_acceleration(process):
    data = np.array(process.accelerations)
    # times = np.array(process.time_arr[:len(data)]) - rospy.Time()
    # times = np.array(list(map(lambda x: x.secs, times)))
   
    smooth_data = ac.smooth_data(process.accelerations)
    # infls = np.where(np.diff(np.sign(smooth_data)))[0]
    plt.plot(data, label='Noisy Data')
    plt.plot(smooth_data, label='Smooth Data')
    # velocity = integrate.simps(smooth_data)
    # distance = integrate.simps(velocity)
    
    # print(velocity)
    # print(distance)
    # plt.plot(velocity, label='Velocity')
    # plt.plot(distance, label='Distance')
    # for i, infl in enumerate(infls,1):
    #     plt.axvline(x=infl, color='k',label='Inflection point %d'%(i))
    plt.legend()
    plt.show()

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

def main():
    IMU_NODE = '/a1_states'
    IMU_MSG_TYPE = 'unitree_legged_msgs/HighState'
    ELEVATOR_FLOOR_NODE = 'elevator_floor'
    ELEVATOR_DOOR_NODE = 'elevator_door'
    DIAGNOSTIC_NODE = '/diagnostics'

    rospy.init_node('elevator_floor', anonymous=True)
    process = CheckElevatorFloorProcess(IMU_NODE, ELEVATOR_FLOOR_NODE, ELEVATOR_DOOR_NODE, DIAGNOSTIC_NODE)

    r = rospy.Rate(100)
    duration = 1

    # only calculate velocity and displacement every 5 secs
    # rospy.Timer(rospy.Duration(duration), process.publish_once_from_queue)
    while not rospy.is_shutdown():
        # process.publish_once_from_queue()
        r.sleep()

    plot_acceleration(process)

if __name__ == '__main__':
    main()
