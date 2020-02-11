#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 12:25:39 2019

@author: mads


This script demos receiving images from the ASUS camera on the Turtlebot. 
Images are converted to numpy arrays in the imagecallback-funciton

"""


import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

cvb = CvBridge()

def msg_to_numpy(data, encoding="16UC1"):
    """Extracts image data from Image message.
    Args:
        data (sensor_msgs/Image): The ROS Image message, exactly as passed
            by the subscriber to its callback.
    Returns:
        The image, as a NumPy array.
    """
    try:
        raw_img = cvb.imgmsg_to_cv2(data, str(encoding))
    except CvBridgeError as err:
        print(err)

    return raw_img

def numpy_to_msg(img):
    """Builds a Image message from a NumPy array.
    Args:
        img (np.array): A NumPy array containing the RGB image data.
    Returns:
        A sensor_msgs/Image containing the image data.
    """
    try:
        data = cvb.cv2_to_imgmsg(img, "rgb8")
    except CvBridgeError as err:
        print(err)

    return data


def imagecallback(data, sensor='frame'):
    img = msg_to_numpy(data, data.encoding)

    # do something here with the image
    cv2.imshow(winname=sensor, mat=img)
    
    cv2.waitKey(1)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imagelistener', anonymous=True)

    rospy.Subscriber("/camera/rgb/image_raw", Image, imagecallback)
    
    rospy.Subscriber("/camera/depth/image_raw", Image, imagecallback, ('depth'))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()