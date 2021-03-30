#!/usr/bin/env python3

import rospkg
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
import cv2
from scipy.ndimage import filters
import message_filters
import numpy as np
import math


class Lidepth:
    def __init__(self):

        # Creation of the publishers
        self.pointCloud2_pub = rospy.Publisher(
            "/lidepth/PointCloud", PointCloud2, queue_size=1)
        self.img_pub = rospy.Publisher(
            "/lidepth/depth_image", Image, queue_size=1)

        # Creation of the subscribers
        self.scan_sub = message_filters.Subscriber("/scan", LaserScan)
        self.pointCloud2_sub = message_filters.Subscriber(
            "/monodepth/pointcloud")

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.pointCloud2_sub, self.scan_sub], 10, 0.1)
        ts.registerCallback(self.callback)

    def callback(self, pointCloud2_sync, scan_sync):

        # Process of the lidar data
        ranges = self.range_filter(scan_sync)

        return 1

    def range_filter(self, scan_sync):

        range_data = np.array(scan_sync.ranges, np.float32)
        angle_min = scan_sync.angle_min
        angle_max = scan_sync.angle_max
        range_min = scan_sync.range_min
        range_max = scan_sync.range_max
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

    def correction(self, ranges, pointCloud_sync):

        U = 3280  # Horizontal number of pixels
        V = 2464  # Vertical number of pixels of the camera sensor

        pointCloud_height = pointCloud_sync.height
        pointCloud_width = pointCloud_sync.width

        # Projection of the lidar data to the 2D pointCloud plane

        Pl = np.array([(np.multiply(-np.sin(ranges[1, :]), ranges[0, :])),
                       np.zeros(len(ranges[0, :])),
                       np.multiply(np.cos(ranges[1, :]), ranges[0, :])], np.float32)

        # Translation vector of the lidar regarding the camera position.
        t = np.array([[0, -0.048, -0.00]], np.float32).T

        # Rotation matrix of the lidar regarding the camera position
        rotationAngle = math.radians(3.8)

        R = np.array([[math.cos(rotationAngle), 0, math.sin(rotationAngle)],
                      [0, 1, 0],
                      [-math.sin(rotationAngle), 0, math.cos(rotationAngle)]], np.float32)

        Pc = R.dot(Pl)+t
        a = 2714.2857  # Focal length in meters
        s = 0  # Skew constant of the camera, here 0 'cause the distortion of the camera is already corrected in the raspicam_node
        u0 = U/2  # int(len(image_np[1, :])/2)
        v0 = V/2  # int(len(image_np[0, :])/2)
        # Camera plane conversion matrix H
        H = np.array([[a, s, u0],
                      [0, a, v0],
                      [0, 0, 1]], np.float32)
        P = H.dot(Pc)
        UV = np.array([np.divide(P[0, :], P[2, :]),
                       np.divide(P[1, :], P[2, :])], np.float32)

        # Get the prixel position on the camera sensor associated to the corresponding lidar depth
        UVZ = np.vstack((UV, P[2, :]))

    def valmap(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def main(args):
    rospy.init_node("lidepth")

    corrected_depth = Lidepth()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image depth sensing module")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
