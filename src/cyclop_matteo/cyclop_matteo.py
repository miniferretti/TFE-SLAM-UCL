#!/usr/bin/env python
   2 """OpenCV feature detectors with ros CompressedImage Topics in python.
   3 
   4 This example subscribes to a ros topic containing sensor_msgs 
   5 CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
   6 then detects and marks features in that image. It finally displays 
   7 and publishes the new image - again as CompressedImage topic.
   8 """
   9 __author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
  10 __version__=  '0.1'
  11 __license__ = 'BSD'
  12 # Python libs
  13 import sys, time
  14 
  15 # numpy and scipy
  16 import numpy as np
  17 from scipy.ndimage import filters
  18 
  19 # OpenCV
  20 import cv2
  21 
  22 # Ros libraries
  23 import roslib
  24 import rospy
  25 
  26 # Ros Messages
  27 from sensor_msgs.msg import CompressedImage
  28 # We do not use cv_bridge it does not support CompressedImage in python
  29 # from cv_bridge import CvBridge, CvBridgeError
  30 
  31 VERBOSE=False
  32 
  33 class image_feature:
  34 
  35     def __init__(self):
  36         '''Initialize ros publisher, ros subscriber'''
  37         # topic where we publish
  38         self.image_pub = rospy.Publisher("/output/image_raw/compressed",
  39             CompressedImage)
  40         # self.bridge = CvBridge()
  41 
  42         # subscribed Topic
  43         self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed_image",
  44             CompressedImage, self.callback,  queue_size = 1)
  45         if VERBOSE :
  46             print "/raspicam_node/image/compressed_image"
  47 
  48 
  49     def callback(self, ros_data):
  50         '''Callback function of subscribed topic. 
  51         Here images get converted and features detected'''
  52         if VERBOSE :
  53             print 'received image of type: "%s"' % ros_data.format
  54 
  55         #### direct conversion to CV2 ####
  56         np_arr = np.fromstring(ros_data.data, np.uint8)
  57         image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
  58         #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
  59         
  60         #### Feature detectors using CV2 #### 
  61         # "","Grid","Pyramid" + 
  62         # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
  63         method = "GridFAST"
  64         feat_det = cv2.FeatureDetector_create(method)
  65         time1 = time.time()
  66 
  67         # convert np image to grayscale
  68         featPoints = feat_det.detect(
  69             cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
  70         time2 = time.time()
  71         if VERBOSE :
  72             print '%s detector found: %s points in: %s sec.'%(method,
  73                 len(featPoints),time2-time1)
  74 
  75         for featpoint in featPoints:
  76             x,y = featpoint.pt
  77             cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
  78         
  79         cv2.imshow('cv_img', image_np)
  80         cv2.waitKey(2)
  81 
  82         #### Create CompressedIamge ####
  83         msg = CompressedImage()
  84         msg.header.stamp = rospy.Time.now()
  85         msg.format = "jpeg"
  86         msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
  87         # Publish new image
  88         self.image_pub.publish(msg)
  89         
  90         #self.subscriber.unregister()
  91 
  92 def main(args):
  93     '''Initializes and cleanup ros node'''
  94     ic = image_feature()
  95     rospy.init_node('cyclop_matteo', anonymous=True)
  96     try:
  97         rospy.spin()
  98     except KeyboardInterrupt:
  99         print "Shutting down ROS Image feature detector module"
 100     cv2.destroyAllWindows()
 101 
 102 if __name__ == '__main__':
 103     main(sys.argv)