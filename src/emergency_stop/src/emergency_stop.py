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
import time
import math
import time
sys.setrecursionlimit(10**8)


class Emergency_stop:
    def __init__(self):

        # Parameters

        self.min_dist = rospy.get_param("~min_dist", 1)
        self.threshold = rospy.get_param("~threshold", 0.1)
        self.queue_size = rospy.get_param("~queue_size", 1)

        self.topic_scan = rospy.get_param("~topic_lidar_data", "/scan")
        self.topic_emergency_statsus = rospy.get_param(
            "~emergency_topic", "/emergency_message")

        # Publishers
        self.pub_emergency_status = rospy.Publisher(
            self.topic_emergency_statsus, EmergencyStatus, queue_size=1)

        # Subscribers
        self.sub_scan = rospy.Subscriber(
            self.topic_scan, LaserScan, self.callback)

    def range_filter(self, scan_sync):

        range_data = np.array(scan_sync.ranges, np.float32)
        angle_min = scan_sync.angle_min
        angle_max = scan_sync.angle_max
        range_min = np.min(range_data)
        range_max = np.max(range_data)
        angle_increment = scan_sync.angle_increment
        N = int((angle_max-angle_min)/angle_increment)
        angle_data = np.linspace(angle_min, angle_max, num=(N+1)) + math.pi
        # print(len(angle_data))
        # print(len(range_data))

        ranges = np.array([range_data, angle_data], np.float32)

        ranges[0, ranges[0, :] > range_max] = range_max
        ranges[0, ranges[0, :] < range_min] = range_min

        # for i in range(len(ranges[0, :])):
        #    print((ranges[0, i], ranges[1, i]))

        return ranges

    def callback(self, scan):

        ranges = self.range_filter(scan)
        msg = EmergencyStatus()
        msg.header = rospy.Time.now()

        obs = ranges[:, ranges[0, :] <= self.min_dist]

        n_obs = 0
        if obs.size != 0:
            n = 0
            prev = None
            n_obs = 1
            for col in obs:
                if n == 0:
                    prev = col
                    n += 1
                else:

                    dist = math.sqrt(math.pow(
                        col[0], 2) + math.pow(prev[0], 2)-2*col[0]*prev[0]*math.cos(col[1]-prev[1]))

                    if dist > self.threshold:
                        n_obs += 1
                    prev = col
            msg.obstacle_detected = True
            msg.num_obs = n_obs
            msg.ranges = obs[0]
            msg.angles = obs[1]
        else:
            msg.obstacle_detected = False
        print("Number of obstacle detected {}".format(obs))
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
