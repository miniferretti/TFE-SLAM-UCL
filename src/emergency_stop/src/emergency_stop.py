#########################################################
# Failsafe node to avoid unseen obstacles by
# the depth perception algorithm (Monodepth_adabin)
# of the drone.
# This node subscribes to scan messages from the LiDar
# sensor.
#
# Author: Matteo Ferretti
# Date: 25/7/2021
#########################################################


from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, LaserScan,
                             PointCloud2, PointField)
from emergency_stop.msg import EmergencyStatus
import rospkg
import rospy
import numpy as np
import sys


class Emergency_stop:
    def __init__(self):

        # Parameters

        self.min_dist = rospy.get_param("~min_dist", 1)
        self.queue_size = rospy.get_param("~queue_size", 2)

        self.topic_scan = rospy.get_param("~topic_lidar_data", "/scan")
        self.topic_emergency_statsus = rospy.get_param(
            "~emergency_topic", "/emergency_message")

        # Publishers
        self.pub_emergency_status = rospy.Publisher(
            self.topic_emergency_statsus, EmergencyStatus, queue_size=1)

        # Subscribers
        self.sub_scan = rospy.Subscriber(
            self.topic_scan, LaserScan, self.callback)

    def callback(self, scan):

        msg = None

        # Publish the emergency message
        self.pub_emergency_status.Publish(msg)


def main(args):
    rospy.init_node("emergency_stop")

    toDo = Emergency_stop()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image depth sensing module")


if __name__ == "__main__":
    main(sys.argv)
