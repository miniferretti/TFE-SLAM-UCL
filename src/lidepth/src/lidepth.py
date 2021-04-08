#!/usr/bin/env python3

import rospkg
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
import cv2
from scipy.ndimage import filters
import message_filters
import numpy as np
import math
import sys

#from monodepth import monodepth


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
            "/monodepth/pointcloud",PointCloud2, queue_size=1)

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.pointCloud2_sub, self.scan_sub], 10, 0.1)
        ts.registerCallback(self.callback)

        

    def callback(self, pointCloud2_sync, scan_sync):

        # Process of the lidar data
        ranges = self.range_filter(scan_sync,pointCloud2_sync)

        # Generate Corrected Point cloud 
        cloudCorrected_msg = self.correction(ranges,pointCloud2_sync)
        self.pub_pointCloud2.publish(cloudCorected_msg)
    
        return 1

    def range_filter(self, scan_sync, pointCloud2_sync):

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

    def correction(self, ranges, pointCloud2_sync):


        U = 3280  # Horizontal number of pixels
        V = 2464  # Vertical number of pixels of the camera sensor


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

        # Get the pixel position on the camera sensor associated to the corresponding lidar depth
        UVZ = np.vstack((UV, P[2, :]))

        #for i in range(len(UVZ[0, :])):
            #u = UVZ[0, i]
            #v = UVZ[1, i]
            #z = UVZ[2, i]
            #if (u <= U) and (v <= V):
                #if (u >= 0) and (v >= 0) and (z >= 0):


        ###################################################
        # If someone wants to reorder information 
        #in pointcould in [z,x,y,R,G,B]
        ###################################################

        #data = pointCloud2_sync.data

        #pointCloudArray = np.zeros((len(data),6))
        #for i in range(len(data)):
            #pointCloudArray[i ; 1] = data[i]        // x
            #pointCloudArray[i ; 2] = data[i + 1]    // y
            #pointCloudArray[i ; 0] = data[i + 2]    // z
            #pointCloudArray[i ; 5] = data[i + 3]    // R
            #pointCloudArray[i ; 4] = data[i + 4]    // G             
            #pointCloudArray[i ; 3] = data[i + 5]    // B
            #i += 6
        ###################################################

   
        #Creation of correctedPointCloud

        msg = PointCloud2()

        msg.header.stamp = pointCloud2_sync.header.stamp
        msg.header.frame_id = pointCloud2_sync.header.frame_id
        msg.header.seq = pointCloud2_sync.header.seq

        msg.width = pointCloud2_sync.width
        msg.height = pointCloud2_sync.height

        data = np.zeros((pointCloud2_sync.width * pointCloud2_sync.height * 6), dtype=np.float32)
        data = pointCloud2_sync.data


        # Correction with LiDAR Data

        u = pointCloud2_sync.width
        v = pointCloud2_sync.height
   
        for i in range(len(UV[0, :])):
            u = UV[0, i]
            v = UV[1, i]
            if (u <= U) and (v <= V):
                if (u >= 0) and (v >= 0) and (P[2, i] >= 0):
                    u_pointCloud = self.valmap(u, 0, U, 0, pointCloud2_sync.width)
                    v_pointCloud = self.valmap(v, 0, V, 0, pointCloud2_sync.height)
                    ipixel = int((v_pointCloud * u) + u_pointCloud)

                    differenceIpixel = data[(ipixel*6)+2] - P[2, i]
                    print('difference in pixel at [ "%s" ; "%s" ] is : "%s" ', data[(ipixel*6)], data[(ipixel*6)+1],differenceIpixel)
                    
                    data[(ipixel*6)+2] = P[2, i]

                    # Stores the LiDar pixels kept on the image
                    #P_real = np.append(P_real, P[:, i])

        print("Point cloud adjusted with Lidar Data")
        

        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * height * width
        msg.is_dense = True
        msg.data = data.tobytes()

        correctedCloud = msg

        return correctedCloud


        ###################################################
        # Correction des données des lignes
        ###################################################


        ###################################################



    
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
