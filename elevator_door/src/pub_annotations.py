#!/usr/bin/env python
"""
Publishes annotated image from door_state
"""

import rospy
import numpy as np
import ros_numpy
import tf
import cv2

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from elevator_door.msg import ElevatorDoorState


def callback(door_state):
    background_img = np.zeros((480, 640, 3)).astype(np.uint8)

    # elevator frame
    cv2.rectangle(background_img, (door_state.lbound, 0), (door_state.rbound, 480), (0,0,255), 5, cv2.LINE_AA)
    # elevator opening
    cv2.rectangle(background_img, (door_state.lopening, 0), (door_state.ropening, 480), (0,255,0), 3, cv2.LINE_AA)

    # publish line-annotated image
    img_msg = ros_numpy.msgify(Image, background_img, encoding='rgb8')
    hough_pub = rospy.Publisher('elevator/annotated_img', Image, queue_size=10)
    hough_pub.publish(img_msg)

    

if __name__ == '__main__':
    rospy.init_node('door_viz_node')
    state_sub = rospy.Subscriber('elevator/door_state', ElevatorDoorState, callback)
    rospy.spin()
