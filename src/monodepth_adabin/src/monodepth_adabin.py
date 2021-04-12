#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
import sys
import time
import numpy as np
import cv2
import rospkg
import struct
import rospy
from scipy.ndimage import filters
from models import UnetAdaptiveBins
import models_io
from infer import InferenceHelper
from predict import depth_norm

MIN_DEPTH = 1e-3
MAX_DEPTH_NYU = 10
MAX_DEPTH_KITTI = 80

N_BINS = 256


class MonoDepth_adabin:
    def __init__(self):

        print("Hello world")

        # Get parameters
        self.debug = rospy.get_param("~debug", False)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.device = rospy.get_param("~device", 'cpu')

        self.topic_color = rospy.get_param(
            "~topic_color", "/raspicam_node/image")
        self.topic_depth = rospy.get_param(
            "~topic_depth", "/monodepth_adabin/image_depth")
        self.topic_pointcloud = rospy.get_param(
            "~topic_pointcloud", "/monodepth_adabin/pointcloud")
        self.topic_camera_info = rospy.get_param(
            "~topic_camera_info", "/raspicam_node/camera_info")

        self.min_depth = rospy.get_param("~min_depth", MIN_DEPTH)
        self.max_depth = rospy.get_param("~max_depth", MAX_DEPTH_NYU)

        # Load model into GPU / CPU
        self.infer_helper = InferenceHelper(
            dataset='nyu', device=self.device, MAX_DEPTH_NYU=self.max_depth, MIN_DEPTH=self.min_depth)

        # Publishers
        self.pub_image_depth = rospy.Publisher(
            self.topic_depth, Image, queue_size=1)
        self.pub_pointcloud = rospy.Publisher(
            self.topic_pointcloud, PointCloud2, queue_size=1)
        self.counter = 0

        # Subscribers
        self.bridge = CvBridge()
        self.sub_image_raw = rospy.Subscriber(
            self.topic_color, Image, self.image_callback)
        self.sub_camera_info = rospy.Subscriber(
            self.topic_camera_info, CameraInfo, self.camera_info_callback)
        self.camera_info = None

        print("Hello world")

    # Create a sensor_msgs.PointCloud2 from the depth and color images provided
    def create_pointcloud_msg(self, depth, color):

        height, width = depth.shape
        P = self.camera_info.P
        print(P)

        # Resize color to match depth
        img = cv2.resize(color, (width, height))

        # Point cloud data list
        points = []

        # Iterate images and build point cloud
        for v in range(height):
            for u in range(width):
                x, y, z = self.convert_from_uvd(self.valmap(
                    u, 0, width, 0, 1280), self.valmap(v, 0, height, 0, 960), depth[v, u], P)
                b = img[v, u, 0]  # b
                g = img[v, u, 1]  # g
                r = img[v, u, 2]  # r
                a = 255
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                pt = [x, y, z, rgb]
                points.append(pt)

        # Fields of the point cloud
        fields = [
            PointField("y", 0, PointField.FLOAT32, 1),
            PointField("z", 4, PointField.FLOAT32, 1),
            PointField("x", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1),
        ]

        header = Header()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, points)
        pc2.header.stamp = rospy.Time.now()

        return pc2

    def convert_from_uvd(self, u, v, z, P):
        fx = P[0]
        fy = P[5]
        x = u * z/fx
        y = v * z/fy
        return x, y, z

    def valmap(self, value, istart, istop, ostart, ostop):
        return int(ostart + (ostop - ostart) * ((value - istart) / (istop - istart)))

        # Callback to recieve and store the camera_info parameters

    def camera_info_callback(self, data):
        self.camera_info = data

        # Callback to receive and process image published.
        #
        # After processing it publishes back the estimated depth result

    def image_callback(self, data):

        print("New frame processed")
        # Convert message to opencv image
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Display image
        image = cv2.rotate(image, cv2.ROTATE_180)
        if self.debug:
            cv2.imshow("Image", image)
            cv2.waitKey(1)

        # Get image data as a numpy array to be passed for processing.
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        img = cv2.resize(img, (640, 480))

        # Predict depth image
        bin_centers, true_depth = self.infer_helper.predict_pil(img)

        depth = np.clip(depth_norm(true_depth.squeeze(), max_depth=MAX_DEPTH_NYU), MIN_DEPTH,
                        MAX_DEPTH_NYU) / MAX_DEPTH_NYU  # Ligne de code a valider

        true_depth = true_depth.squeeze()

        # Display depth
        if self.debug:
            cv2.imshow("Result", depth)
            cv2.waitKey(1)

        # Publish depth image
        depth = 255 * depth
        self.pub_image_depth.publish(
            self.bridge.cv2_to_imgmsg(depth.astype(np.uint8), "mono8"))

        # Generate Point cloud
        cloud_msg = self.create_pointcloud_msg(true_depth, image)
        self.pub_pointcloud.publish(cloud_msg)

        # Increment counter
        self.counter += 1


def main(args):
    rospy.init_node("monodepth_adabin")

    depth = MonoDepth_adabin()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image depth sensing module")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
