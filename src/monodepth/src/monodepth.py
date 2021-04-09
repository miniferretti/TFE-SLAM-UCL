#!/usr/bin/env python3

from layers import BilinearUpSampling2D
from predict import predict
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
import sys
import time
import numpy as np
import cv2
import rospkg
import rospy

#from lidepth.source.lidepth import lidepth


# import keras
import tensorflow as tf
print("Hello world")
# from tensorflow import keras


print("Hello world")
tf.compat.v1.disable_eager_execution()


class MonoDepth:
    def __init__(self):

        print("Hello world")

        # Setup tensorflow session
        self.session = tf.compat.v1.keras.backend.get_session()
        self.init = tf.compat.v1.global_variables_initializer()
        self.session.run(self.init)

        # Get parameters
        self.debug = rospy.get_param("~debug", True)
        self.frame_id = rospy.get_param("~frame_id", "map")

        self.topic_color = rospy.get_param(
            "~topic_color", "/raspicam_node/image")
        self.topic_depth = rospy.get_param(
            "~topic_depth", "/monodepth/image_depth")
        self.topic_pointcloud = rospy.get_param(
            "~topic_pointcloud", "/monodepth/pointcloud")

        self.min_depth = rospy.get_param("~min_depth", 10)
        self.max_depth = rospy.get_param("~max_depth", 1000)
        self.batch_size = rospy.get_param("~batch_size", 1)
        self.model_file = rospy.get_param("~model_file", "/models/nyu.h5")

        # Read keras model
        self.rospack = rospkg.RosPack()
        self.model_path = self.rospack.get_path("monodepth") + self.model_file

        # Custom object needed for inference and training
        self.start = time.time()
        self.custom_objects = {"BilinearUpSampling2D": BilinearUpSampling2D,
                               "depth_loss_function": self.depth_loss_function}

        # Load model into GPU / CPU
        self.model = tf.keras.models.load_model(
            self.model_path, custom_objects=self.custom_objects, compile=False)
        self.model._make_predict_function()

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

    # Loss function for the depth map
    def depth_loss_function(self, y_true, y_pred):
        theta = 0.1
        max_depth_val = self.max_depth / self.min_depth

        # Point-wise depth
        l_depth = tf.keras.backend.mean(
            tf.keras.backend.abs(y_pred - y_true), axis=-1)

        # Edges
        dy_true, dx_true = tf.image.image_gradients(y_true)
        dy_pred, dx_pred = tf.image.image_gradients(y_pred)
        l_edges = tf.keras.backend.mean(tf.keras.backend.abs(
            dy_pred - dy_true) + tf.keras.backend.abs(dx_pred - dx_true), axis=-1)

        # Structural similarity (SSIM) index
        l_ssim = tf.keras.backend.clip(
            (1 - tf.image.ssim(y_true, y_pred, max_depth_val)) * 0.5, 0, 1)

        # Weights
        w1 = 1.0
        w2 = 1.0
        w3 = theta

        return (w1 * l_ssim) + (w2 * tf.keras.backend.mean(l_edges)) + (w3 * tf.keras.backend.mean(l_depth))

    # Create a sensor_msgs.PointCloud2 from the depth and color images provided
    #
    # It ignores are camera parameters and assumes the images to be rectified


    def create_pointcloud_msg(self, depth, color):
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.header.seq = self.counter

        height, width, c = depth.shape

        #height = int(height/2)
        #width = int(width/2)

        # Resize color to match depth
        img = cv2.resize(color, (width, height))

        # Point cloud data numpy array
        i = 0
        data = np.zeros((height * width * 6), dtype=np.float32)

        # Message data size
        msg.height = 1
        msg.width = width * height

        #msg.height = height
        #msg.width = width

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

        print('***  MonoDepth  ****')
        print('data shape is ', data.shape, '\n')
        size = sys.getsizeof(data)
        print('The size is ', size, '\n')
        print(type(data[0]))
        for iprinting in range(6):
            print(data[(6*100)+iprinting], '\n')

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

    # Callback to receive and process image published.
    #
    # After processing it publishes back the estimated depth result
    def image_callback(self, data):
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

        # Cropping the Image
        img = img[(616):(1848), 0:3280]   # Image : 3280 * 2464

        #img = img[(924):(1386), 0:3280]  
        #img = img[(2464/4):((2464*3)/4), 0:3280]   # Image : 3280 * 2464
        img = cv2.resize(img, (640, 480))

        arr = np.clip(np.asarray(img, dtype=np.float32) / 255, 0, 1)

        # Predict depth image
        with self.session.as_default():
            with self.session.graph.as_default():
                result = predict(self.model, arr, self.min_depth,
                                 self.max_depth, self.batch_size)

        # Resize and reshape output
        depth = result.reshape(result.shape[1], result.shape[2], 1)
        # Display depth`
        if self.debug:
           #depthc = depth*255
            #depthc = cv2.applyColorMap(depthc.astype(int), cv2.COLORMAP_PLASMA)
            cv2.imshow("Result", depth)
            cv2.waitKey(1)

        # Publish depth image
        depth = 255 * depth
        self.pub_image_depth.publish(
            self.bridge.cv2_to_imgmsg(depth.astype(np.uint8), "mono8"))

        # Generate Point cloud
        cloud_msg = self.create_pointcloud_msg(depth, image)
        self.pub_pointcloud.publish(cloud_msg)

        # Increment counter
        self.counter += 1


def main(args):
    rospy.init_node("monodepth")

    depth = MonoDepth()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image depth sensing module")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
