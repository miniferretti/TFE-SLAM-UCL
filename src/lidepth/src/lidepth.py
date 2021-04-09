#!/usr/bin/env python3

import rospkg
import rospy
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
import sensor_msgs.point_cloud2 as pc2
import cv2
from scipy.ndimage import filters
import message_filters
import numpy as np
import ctypes
import math
import sys


#DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                  #(PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                  (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
#pftype_to_nptype = dict(type_mappings)
#nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)
 
 # sizes (in bytes) of PointField types
#pftype_sizes = {PointField.INT8: 1, PointField.UINT8: 1, PointField.INT16: 2, PointField.UINT16: 2,
                 #PointField.INT32: 4, PointField.UINT32: 4, PointField.FLOAT32: 4, PointField.FLOAT64: 8}

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
        self.pointCloud2_pub.publish(cloudCorrected_msg)
    
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


    #def fields_to_dtype(self, fields, point_step):
        #Convert a list of PointFields to a numpy record datatype.

        #offset = 0
        #np_dtype_list = []
        #for f in fields:
            #while offset < f.offset:
                # might be extra padding between fields
                #np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
                #offset += 1
 
            #dtype = pftype_to_nptype[f.datatype]
            #if f.count != 1:
                #dtype = np.dtype((dtype, f.count))
 
            #np_dtype_list.append((f.name, dtype))
            #offset += pftype_sizes[f.datatype] * f.count
 
        # might be extra padding between points
        #while offset < point_step:
            #np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            #offset += 1
         
        #return np_dtype_list

    #####################################################
    #########     Imported function    ##################
    #Converts a rospy PointCloud2 message to a numpy recordarray
    #Reshapes the returned array to have shape (height, width), even if the height is 1.
    #The reason for using np.fromstring rather than struct.unpack is speed... especially
    #for large point clouds, this will be <much> faster.
    #####################################################

    #def pointcloud2_to_array(self, cloud_msg, squeeze=True):
   
        # construct a numpy record type equivalent to the point type of this cloud
        #dtype_list = self.fields_to_dtype(cloud_msg.fields, cloud_msg.point_step) 

        # parse the cloud into an array
        #cloud_arr = np.fromstring(cloud_msg.data, dtype_list)

        # remove the dummy fields that were added
        #cloud_arr = cloud_arr[
            #[fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

        #if squeeze and cloud_msg.height == 1:
            #return np.reshape(cloud_arr, (cloud_msg.width,))
        #else:
            #return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

        #return dtype_list
    #####################################################
    #####################################################

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
        print('******************************************** \n ')
        print('Here we are! \n ')
        print('******************************************** \n ')

        msg = PointCloud2()

        msg.header.stamp = pointCloud2_sync.header.stamp
        msg.header.frame_id = pointCloud2_sync.header.frame_id
        msg.header.seq = pointCloud2_sync.header.seq

        msg.width = pointCloud2_sync.width
        msg.height = pointCloud2_sync.height

        print('pointCloud2_sync.width is %s \n' % (pointCloud2_sync.width))
        print('pointCloud2_sync.height is %s \n' % (pointCloud2_sync.height))


        #data = np.zeros((pointCloud2_sync.width * pointCloud2_sync.height * 6), dtype=np.float32)
        #data = pointCloud2_sync.data
        #print('data shape is %s \n', data.shape)

        #data = self.pointcloud2_to_array(pointCloud2_sync)

        ## Try with pc2.read_points

        pc = pc2.read_points(pointCloud2_sync, skip_nans=True, field_names=("x", "y", "z", "r", "g", "b"))

        data_list = []
        for p in pc:
            data_list.append([p[0],p[1],p[2],p[3],p[4],p[5]])
        
        data = np.array(data_list)
        print('data shape is %s \n' % (data.shape))

        
        

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

                    #differenceIpixel = data[(ipixel*6)+2] - P[2, i]
                    #print('difference in pixel at [ "%s" ; "%s" ] is : "%s" ', data[(ipixel*6)], data[(ipixel*6)+1],differenceIpixel)
                    
                    #data[(ipixel*6)+2] = P[2, i]

                    # Stores the LiDar pixels kept on the image
                    #P_real = np.append(P_real, P[:, i])

        print("Point cloud adjusted with Lidar Data")
        

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
        msg.row_step = msg.point_step * msg.height * msg.width
        msg.is_dense = True
        msg.data = data.tobytes()

        correctedCloud = msg

        print('******************************************** \n ')

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
