#!/usr/bin/env python
"""
Authors(s): Christy Koh
A test for tuning different edge detection algorithms.
"""

import rospy
import numpy as np
import cv2
import ros_numpy
from sensor_msgs.msg import Image


def apply_filters(image):
    # convert to numpy
    img = ros_numpy.numpify(image)
    gray = cv2.convertScaleAbs(img)

    # apply filters
    sobel = cv2.Sobel(gray, -1, 0, 1)
    # sobel_uint8 = cv2.threshold(sobel, 200, 255, 0)
    canny = cv2.Canny(img, 200, 250)

    # convert to Image msg
    sobel_msg = ros_numpy.msgify(Image, sobel, encoding='rgb8')
    canny_msg = ros_numpy.msgify(Image, canny, encoding='mono8')

    return sobel_msg, canny_msg

def get_callback(pub1, pub2):
    """
    Returns callback that applies filters to input image and
        publishes to the appropriate topic.
    """
    def callback(image):
        sobel_img, canny_img = apply_filters(image)
        pub1.publish(sobel_img)
        pub2.publish(canny_img)

    return callback

def main():
    sobely_pub = rospy.Publisher('sobely', Image, queue_size=10)
    canny_pub = rospy.Publisher('canny', Image, queue_size=10)
    sobely_depth_pub = rospy.Publisher('sobely_depth', Image, queue_size=10)
    canny_depth_pub = rospy.Publisher('canny_depth', Image, queue_size=10)

    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, get_callback(sobely_pub, canny_pub))
    # depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, get_callback(sobely_depth_pub, canny_depth_pub))
        
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('filter_test')
    main()
