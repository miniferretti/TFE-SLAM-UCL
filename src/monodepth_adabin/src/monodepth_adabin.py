#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo, LaserScan
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
import message_filters
from scipy.ndimage import filters
import matplotlib.pyplot as plt


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
        self.topic_laserScan = rospy.get_param("~topic_lidar_data", "/scan")

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
        self.sub_camera_info = rospy.Subscriber(
            self.topic_camera_info, CameraInfo, self.camera_info_callback)

        self.sub_image_raw = message_filters.Subscriber(
            self.topic_color, Image)
        self.sub_laserScan = message_filters.Subscriber(
            self.topic_laserScan, LaserScan)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_image_raw, self.sub_laserScan], 10, 0.1)
        ts.registerCallback(self.image_lidar_callback)

        self.camera_info = None

        print("Hello world")


    def depth_correction(self, ranges, depth)    

        U = 3280  # Horizontal number of pixels
        V = 2464  # Vertical number of pixels of the camera sensor

        image_height, image_width = depth.shape

        Pl = np.array([(np.multiply(-np.sin(ranges[1, :]), ranges[0, :])),
                       np.zeros(len(ranges[0, :])),
                       np.multiply(np.cos(ranges[1, :]), ranges[0, :])], np.float32)

        # Translation vector between the camera and the lidar (lidar --> Camera translation) everything in meters
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

        for i in range(len(UV[0, :])):
            u = UV[0, i]
            v = UV[1, i]

            if (u <= U) and (v <= V):
                if (u >= 0) and (v >= 0) and (P[2, i] >= 0):
                    u_real = self.valmap(u, 0, U, 0, image_width)
                    v_real = self.valmap(v, 0, V, 0, image_height)

                    differenceDepth = depth[v_real ; u_real] - P[2, i]
                    depth[v_real ; u_real] = P[2, i]

                    for hh in range(image_height)
                        depth[hh ; u_real] = depth[hh ; image_height] + differenceDepth *((image_height - abs(v_real - hh))/image_height)



        return depth

    # Create a sensor_msgs.PointCloud2 from the depth and color images provided
    def create_pointcloud_msg(self, depth, color):

        height, width = depth.shape
        P = self.camera_info.P
        print(P)
        # Resize color to match depth
        img = np.fliplr(cv2.resize(color, (width, height)))
        depth = np.fliplr(depth)
        # Point cloud data list
        points = []

        # Iterate images and build point cloud
        for v in range(height):
            for u in range(width):
                x, y, z = self.convert_from_uvd(self.valmap(
                    (u - (width / 2)), 0, width, 0, 1280), self.valmap((-v + (height / 2)), 0, height, 0, 960), depth[v, u], P)
                b = img[v, u, 0]  # b
                g = img[v, u, 1]  # g
                r = img[v, u, 2]  # r
                a = 255
                if abs(x) < 0.01 and abs(y) < 0.01:
                    print(x)
                    print(y)
                    print(z)
                    b = 255
                    g = 0
                    r = 0
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
        pixtometer = 1
        fx = P[0] * pixtometer
        fy = P[5] * pixtometer
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

    def image_lidar_callback(self, image_sync, scan_sync):

        print("New frame processed")
        # Convert message to opencv image
        try:
            image = self.bridge.imgmsg_to_cv2(image_sync, "bgr8")
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
        cm = plt.get_cmap('magma')
        depth = cm(depth)
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
