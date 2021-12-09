#!/usr/bin/env python
"""
Author(s): Christy Koh
Main elevator detection node, subscribes to pointcloud
and raw RGB image, and publishes state of elevator door.

Some parts adapted from EECS106A Fall 2021, Lab 6.
"""

from __future__ import print_function
from collections import deque

import rospy
import numpy as np
import message_filters
import ros_numpy
import tf

from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from elevator_door.msg import ElevatorDoorState
import matplotlib.pyplot as plt

from door_state import ElevatorDoorTracker

def get_camera_matrix(camera_info_msg):
    return np.array(camera_info_msg.K).reshape((3,3))


class ElevatorImageProcess:
    """ 
    Wraps the processing of a pointcloud and RGB image, publishing
    to a segmented image topic and message state topic.

    """

    def __init__(self, points_sub_topic, 
                image_sub_topic,
                cam_info_topic, 
                image_pub_topic,
                state_pub_topic):
        
        self.messages = deque([], 5)

        self.state_tracker = ElevatorDoorTracker()
        self.calibrate_tracker = True

        # init subscribers
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self.listener = tf.TransformListener()
        
        # init publishers
        self.state_pub = rospy.Publisher(state_pub_topic, ElevatorDoorState, queue_size=10)
        
        # synchronize input from all three subscribers
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          20, 0.1, allow_headerless=True)
        ts.registerCallback(self.sensor_callback)

        # init service
        rospy.Service('/calibrate_depth', Empty, self.reset_service_callback)
    
    def sensor_callback(self, points_msg, image, info):
        try:
            # print(points_msg.header.stamp)
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            points = ros_numpy.numpify(points_msg)
            #print(points_msg.fields)
        except Exception as e:
            rospy.logerr(e)
            return
        self.messages.appendleft((points, rgb_image, intrinsic_matrix))
    
    def reset_service_callback(self, msg):
        """Flags tracker to calibrate closed depth."""
        self.calibrate_tracker = True
        return EmptyResponse()

    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()
            try:
                trans, rot = self.listener.lookupTransform(
                                                       '/camera_color_optical_frame',
                                                       '/camera_depth_optical_frame',
                                                       rospy.Time(0))
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException):
                return

            # calibrate tracker if necessary
            if self.calibrate_tracker:
                self.state_tracker.set_door_depth_average(points)
                self.calibrate_tracker = False

            door_state = self.state_tracker.process_state(points, image, info,
                np.array(trans), np.array(rot), print_state=False)
            # self.state_tracker.publish_annotated_image()

            # publish door state
            self.state_pub.publish(door_state)


if __name__ == '__main__':
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    DEPTH_IMAGE_TOPIC = '/camera/depth/image_rect_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    IMAGE_PUB_TOPIC = '/elevator/image'
    STATE_PUB_TOPIC = '/elevator/door_state'

    print("initializing node")
    rospy.init_node('elevator_node')
    print("initializing process")
    process = ElevatorImageProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, IMAGE_PUB_TOPIC,
                                STATE_PUB_TOPIC)
    r = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()
