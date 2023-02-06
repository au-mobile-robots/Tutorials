#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 12:25:39 2019

@author: Mads Dyrmann


This script demos receiving laserscans from the Turtlebot 3

"""


import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import math

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
        raw_img = cvb.imgmsg_to_cv2(data)
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


def scancallback(data, sensor='frame'):
    print("SCAN!!!!")
    frame = np.zeros((500, 500,3), np.uint8)
    angle = data.angle_min
    for r in data.ranges:
        #change infinite values to 0
        if math.isinf(r) == True:
            r = 0
        #convert angle and radius to cartesian coordinates
        x = math.trunc((r * 30.0)*math.cos(angle + (-90.0*3.1416/180.0)))
        y = math.trunc((r * 30.0)*math.sin(angle + (-90.0*3.1416/180.0)))

        #set the borders (all values outside the defined area should be 0)
        if y > 0 or y < -35 or x<-40 or x>40:
            x=0
            y=0

        cv2.line(frame,(250, 250),(x+250,y+250),(255,0,0),2)
        angle= angle + data.angle_increment 
        cv2.circle(frame, (250, 250), 2, (255, 255, 0))
        cv2.imshow('frame',frame)
        cv2.waitKey(1)
    
def listener():
    print("Setup")
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('imagelistener', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, scancallback)
    
    #rospy.Subscriber("/camera/depth/image_raw", Image, imagecallback, ('depth'))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
