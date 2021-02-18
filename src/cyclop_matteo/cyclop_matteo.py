#!/usr/bin/env python

##http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber

import sys, time

import numpy as np
from scipy.ndimage import filters

import rospy
import roslib
import cv2

from sensor_msgs.msg import CompressedImage

def subscriber():
    sub = rospy.Subscriber('/raspicam_node/image/compressed_image',Image, my_cyclop)
    rospy.spin()

def my_cyclop(my_image):

if __name__ == "__main__":
    rospy.init_node("cyclop_matteo")
    subscriber()
