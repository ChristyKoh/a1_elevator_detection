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

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from elevator_door.msg import ElevatorDoorState
import matplotlib.pyplot as plt

from door_state import ElevatorStateTracker, determine_elevator_state

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
        
        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None

        self.state_tracker = ElevatorStateTracker(queue_len=5)

        # init subscribers
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        # self._bridge = CvBridge()
        # self.listener = tf.TransformListener()
        
        # init publishers
        # self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher(image_pub_topic, Image, queue_size=10)
        self.state_pub = rospy.Publisher(state_pub_topic, ElevatorDoorState, queue_size=10)
        
        # synchronize input from all three subscribers
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    
    def callback(self, points_msg, image, info):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            points = ros_numpy.numpify(points_msg)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((points, rgb_image, intrinsic_matrix))


    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()
            # try:
            #     trans, rot = self.listener.lookupTransform(
            #                                            '/camera_color_optical_frame',
            #                                            '/camera_depth_optical_frame',
            #                                            rospy.Time(0))
            #     rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            # except (tf.LookupException,
            #         tf.ConnectivityException, 
            #         tf.ExtrapolationException):
            #     return
            # points = isolate_object_of_interest(points, image, info, 
            #     np.array(trans), np.array(rot))
            # points_msg = numpy_to_pc2_msg(points)
            # self.points_pub.publish(points_msg)

            # pub_image, _ = determine_elevator_state(image)
            self.state_tracker.apply_hough_line_tfm(image)
            door_state = self.state_tracker.get_state(points, image, info)

            # convert numpy image array to msg
            # img_msg = ros_numpy.msgify(Image, pub_image, encoding='mono8')
            # img_msg.header.stamp = rospy.Time.now()
            # img_msg.header.frame_id = 'camera_depth_optical_frame'

            # publish image and door state string
            # self.image_pub.publish(img_msg)
            self.state_pub.publish(door_state)
            # print("Published state at timestamp:", img_msg.header.stamp.secs)


if __name__ == '__main__':
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    DEPTH_IMAGE_TOPIC = '/camera/depth/image_rect_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    IMAGE_PUB_TOPIC = '/elevator/image'
    STATE_PUB_TOPIC = '/elevator/door_state'

    rospy.init_node('elevator_node')
    process = ElevatorImageProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, IMAGE_PUB_TOPIC,
                                STATE_PUB_TOPIC)
    r = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        process.publish_once_from_queue()
        r.sleep()
