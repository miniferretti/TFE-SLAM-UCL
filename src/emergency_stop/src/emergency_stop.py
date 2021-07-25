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
import rospkg
import rospy
import numpy as np

class Emergency_stop:
    def __init__(self):

        # Parameters
        self.topic_scan = rospy.get_param("~topic_lidar_data","/scan")
        self.min_dist = rospy.get_param("~min_dist",1)
        self.queue_size = rospy.get_param("~queue_size", 2)

        # Publishers
        self.pub_



def main(args):
    rospy.init_node("emergency_stop")

    depth = Emergency_stop()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image depth sensing module")


if __name__ == "__main__":
    main(sys.argv)