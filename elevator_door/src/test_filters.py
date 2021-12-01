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

def apply_sobel(img):
    """
    Given numpy repn of image, apply sobel filter
    """
    gray = cv2.convertScaleAbs(img)
    sobel = cv2.Sobel(gray, -1, 0, 1)
    # sobel_uint8 = cv2.threshold(sobel, 200, 255, 0)
    # convert to Image msg
    sobel_msg = ros_numpy.msgify(Image, sobel, encoding='rgb8')
    return sobel_msg

def apply_depth_sobel(img):
    gray = cv2.convertScaleAbs(img)
    sobel = cv2.Sobel(gray, -1, 0, 1)
    # sobel_uint8 = cv2.threshold(sobel, 200, 255, 0)
    # convert to Image msg
    sobel_msg = ros_numpy.msgify(Image, sobel, encoding='mono8')
    
    return sobel_msg


def apply_canny(img):
    """
    Given numpy repn of image, apply canny filter
    """
    # apply filters
    canny = cv2.Canny(img, 200, 250)
    # convert to Image msg
    canny_msg = ros_numpy.msgify(Image, canny, encoding='mono8')
    return canny_msg

def apply_filters(image):
    """ 
    Apply both filters.
    """
    # convert to numpy
    img = ros_numpy.numpify(image)
    sobel_msg = apply_sobel(img)
    canny_msg = apply_canny(img)

    return sobel_msg, canny_msg

def get_double_callback(pub1, pub2):
    """
    Returns callback that applies two filters to input image and
        publishes to the appropriate topics.
    """
    def callback(image):
        sobel_img, canny_img = apply_filters(image)
        pub1.publish(sobel_img)
        pub2.publish(canny_img)

    return callback

def get_single_callback(pub):
    """
    Returns callback that applies filter to input image and
        publishes to the appropriate topic.
    """   
        img = ros_numpy.numpify(image)
        sobel_msg = apply_depth_sobel(img)
        print("publishing msg")
        pub.publish(sobel_msg)
    return callback

def main():
    sobely_pub = rospy.Publisher('sobely', Image, queue_size=10)
    canny_pub = rospy.Publisher('canny', Image, queue_size=10)
    sobely_depth_pub = rospy.Publisher('sobely_depth', Image, queue_size=10)
    # canny_depth_pub = rospy.Publisher('canny_depth', Image, queue_size=10)

    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, get_double_callback(sobely_pub, canny_pub))
    depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, get_single_callback(sobely_pub))
        
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('filter_test')
    main()
  