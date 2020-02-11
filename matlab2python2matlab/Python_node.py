#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 11 12:25:39 2019

@author: mads
"""


import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


cvb = CvBridge()


def msg_to_numpy(data):
    """Extracts image data from Image message.
    Args:
        data (sensor_msgs/Image): The ROS Image message, exactly as passed
            by the subscriber to its callback.
    Returns:
        The image, as a NumPy array.
    """
    try:
        raw_img = cvb.imgmsg_to_cv2(data, "bgr8")
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
        data = cvb.cv2_to_imgmsg(img, "bgr8")
    except CvBridgeError as err:
        print(err)

    return data



def imagecallback(data):
    # callbackfunction, which is invoked when a message is received. The
    print('Received an image')
    
    img = msg_to_numpy(data)   
    cv2.imshow(winname='frame', mat=img)
    cv2.waitKey(10)
    
    # Do some processing
    img = np.fliplr(img) # flip the image to demonstrate that some processing happended in Python
    
    # publish the image on some topic
    pub = rospy.Publisher('/imagefrompython', Image, queue_size=10)
    pub.publish(numpy_to_msg(img))
    print('Published image')
    
    
    
def imagelistener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print('Starting')
    rospy.init_node('pythonnode', anonymous=True)
    rospy.Subscriber('/imagefrommatlab', Image, imagecallback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





if __name__ == '__main__':
    imagelistener()
