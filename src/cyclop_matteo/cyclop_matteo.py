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

VERBOSE = True


class image_feature:

    def __init__(self):

        self.image_pub = rospy.Publisher(
            "/output/image_raw/compressed", CompressedImage, queue_size=1)

        self.image_sub = message_filters.Subscriber(
            "/raspicam_node/image/compressed", CompressedImage)
        self.scan_sub = message_filters.Subscriber("/scan", LaserScan)

       # self.subscriber = rospy.Subscriber(
        #    "/raspicam_node/image/compressed", CompressedImage, self.callback,  queue_size=1)

        ts = message_filters.TimeSynchronizer(
            [self.image_sub, self.scan_sub], 1)
        ts.registerCallback(self.callback)

        if VERBOSE:
            print("/raspicam_node/image/compressed")

    def callback(self, image_sync, scan_sync):

        if VERBOSE:
            print('received image of type: "%s"' % image_sync.format)

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
        #feat_det = cv2.ORB_create()
        time1 = time.time()

        # convert np image to grayscale
        featPoints = feat_det.detect(
            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        time2 = time.time()
        if VERBOSE:
            print('%s detector found: %s points in: %s sec.' % (method,
                                                                len(featPoints), time2-time1))

        for featpoint in featPoints:
            x, y = featpoint.pt
            cv2.circle(image_np, (int(x), int(y)), 3, (0, 0, 255), -1)

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


def main(args):

    ic = image_feature()
    rospy.init_node('cyclop_matteo', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
