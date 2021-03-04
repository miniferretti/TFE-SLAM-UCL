#!/usr/bin/env python

import sys
import time
import cv2
import numpy as np
import roslib
import rospy
import message_filters
from scipy.ndimage import filters
from sensor_msgs.msg import CompressedImage, LaserScan
import math

VERBOSE = False


class image_feature:

    def __init__(self):

        self.image_pub = rospy.Publisher(
            "/output/image_raw/compressed", CompressedImage, queue_size=1)

        self.image_sub = message_filters.Subscriber(
            "/raspicam_node/image/compressed", CompressedImage)
        self.scan_sub = message_filters.Subscriber("/scan", LaserScan)

       # self.subscriber = rospy.Subscriber(
        #    "/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.scan_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        if VERBOSE:
            print("/raspicam_node/image/compressed")

    def callback(self, image_sync, scan_sync):

        if VERBOSE:
            print('Received image of type: "%s" and number "%s"' %
                  (image_sync.format, image_sync.header.seq))
            print('Received scan number "%s"' % scan_sync.header.seq)

           #### direct conversion to CV2 ####
        np_arr = np.fromstring(image_sync.data, np.uint8)
       # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        image_np = cv2.rotate(image_np, cv2.ROTATE_180)

        #### Feature detectors using CV2 ####
        # "","Grid","Pyramid" +
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        method = "GridFAST"
        feat_det = cv2.FastFeatureDetector_create()
        # feat_det = cv2.ORB_create()
        time1 = time.time()

        # convert np image to grayscale
    #    featPoints = feat_det.detect(
    #        cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
    #    time2 = time.time()
    #    if VERBOSE:
    #        print('%s detector found: %s points in: %s sec.' % (method,
    #                                                            len(featPoints), time2-time1))

     #   for featpoint in featPoints:
     #       x, y = featpoint.pt
     #       cv2.circle(image_np, (int(x), int(y)), 3, (0, 0, 255), -1)

        ################################################################
        ###          Adding lidar data to the image                 ####
        ################################################################

        ranges = self.range_filter(scan_sync)
        image_np = self.lidar_data_to_img(ranges, image_np)

        ################################################################

        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)

        # self.subscriber.unregister()

    # Filters impossible ranges and combines it with angle data
    def range_filter(self, scan_sync):

        range_data = np.array(scan_sync.ranges, np.float32)
        angle_min = scan_sync.angle_min
        angle_max = scan_sync.angle_max
        range_min = scan_sync.range_min
        range_max = scan_sync.range_max
        angle_increment = scan_sync.angle_increment
        N = (angle_max-angle_min)/angle_increment
        angle_data = np.linspace(angle_min, angle_max, num=(N+1)) + math.pi
       # print(len(angle_data))
       # print(len(range_data))

        ranges = np.array([range_data, angle_data], np.float32)

        ranges[0, ranges[0, :] > range_max] = range_max
        ranges[0, ranges[0, :] < range_min] = range_min

        for i in range(len(ranges[0, :])):
            print((ranges[0, i], ranges[1, i]))

        return ranges

    # Converts lidar range data to XYZ coordinates and then projects it to the camera image plane
    def lidar_data_to_img(self, ranges, image_np):

        U = 3280  # Horizontal number of pixels
        V = 2464  # Vertical number of pixels of the camera sensor

        Pl = np.array([np.multiply(-np.sin(ranges[1, :]), ranges[0, :] + 0.4),
                       np.zeros(len(ranges[0, :])),
                       np.multiply(np.cos(ranges[1, :]), ranges[0, :])], np.float32)

        # Translation vector between the camera and the lidar (lidar --> Camera translation) everything in meters
        t = np.array([[0, -0.048, -0.055]], np.float32).T

        # Rotation matrix of the lidar regarding the camera position
        R = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]], np.float32)

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

       # print(UV.shape)

        for i in range(len(UV[0, :])):
            u = UV[0, i]
            v = UV[1, i]
            # print("Vertical Pixel %s and Horizontal pixel number %s", (v, u))

            if (u <= U) and (v <= V):
                if (u >= 0) and (v >= 0) and (P[2, i] >= 0):
                    u_real = self.valmap(u, 0, U, 0, 1280)
                    v_real = self.valmap(v, 0, V, 0, 720)
                    cv2.circle(image_np, (int(u_real), int(v_real)),
                               3, (0, 0, 255), -1)

        return image_np

    def valmap(self, value, istart, istop, ostart, ostop):
        return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))


def main(args):

    rospy.init_node('rp_cyclop', anonymous=True)
    ic = image_feature()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
