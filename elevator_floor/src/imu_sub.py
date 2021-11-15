
"""
Authors(s): Abigail Chin
Functions to extract elevator floor
"""
import numpy as np
import rospy
import IMU.msg

def callback(msg):
    # TODO: calculate distance from accelerometer data
    # double integrate for distance measurement
    # filter accordingly noisy data - determine which filter to use
    # 
    
    return
    
def listener():
    #TODO: find topic that publishes IMU msgs 
    rospy.Subscriber('topic', IMU, callback)
    
    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()
    
    
if __name__ == '__main__':

    rospy.init_node('elevator_floor', anonymous=True)
    listener()
    
