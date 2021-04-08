#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
import sys
import time
import numpy as np
import cv2
import rospkg
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
    #
    # It ignores are camera parameters and assumes the images to be rectified
    def create_pointcloud_msg(self, depth, color):
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.header.seq = self.counter

        height, width = depth.shape

        # Resize color to match depth
        img = cv2.resize(color, (width, height))

        # Point cloud data numpy array
        i = 0
        data = np.zeros((height * width * 6), dtype=np.float32)

        # Message data size
        msg.height = 1
        msg.width = width * height

        # Iterate images and build point cloud
        for y in range(height):
            for x in range(width):
                data[i] = (x - (width / 2)) / 100.0
                data[i + 1] = (-y + (height / 2)) / 100.0
                data[i + 2] = depth[y, x] / 25
                data[i + 3] = float(img[y, x, 0]) / 255.0
                data[i + 4] = float(img[y, x, 1]) / 255.0
                data[i + 5] = float(img[y, x, 2]) / 255.0
                i += 6

        # Fields of the point cloud
        msg.fields = [
            PointField("y", 0, PointField.FLOAT32, 1),
            PointField("z", 4, PointField.FLOAT32, 1),
            PointField("x", 8, PointField.FLOAT32, 1),
            PointField("b", 12, PointField.FLOAT32, 1),
            PointField("g", 16, PointField.FLOAT32, 1),
            PointField("r", 20, PointField.FLOAT32, 1)
        ]

        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * height * width
        msg.is_dense = True
        msg.data = data.tobytes()

        return msg

        # Callback to recieve and store the camera_info parameters

    def camera_info_callback(self, data):
        self.camera_info = data

        # Callback to receive and process image published.
        #
        # After processing it publishes back the estimated depth result

    def image_callback(self, data):
        print("Hello world")
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
        bin_centers, depth = self.infer_helper.predict_pil(img)

        depth = np.clip(depth_norm(depth.squeeze(), max_depth=MAX_DEPTH_NYU), MIN_DEPTH,
                        MAX_DEPTH_NYU) / MAX_DEPTH_NYU  # Ligne de code a valider

        # Display depth
        if self.debug:
            cv2.imshow("Result", depth)
            cv2.waitKey(1)

        # Publish depth image
        depth = 255 * depth
        self.pub_image_depth.publish(
            self.bridge.cv2_to_imgmsg(depth.astype(np.uint8), "mono8"))

        # Generate Point cloud
        cloud_msg = self.create_pointcloud_msg(depth, image)
        self.pub_pointcloud.publish(cloud_msg)

        # Publishes the related camera_info for the creation of the PointCloud2 node
       # self.pub_camera_info.publish(camera_info)
      #  self.pub_color_image.publish(
       #     self.bridge.cv2_to_imgmsg(img.astype(np.uint8), "bgr8"))

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
