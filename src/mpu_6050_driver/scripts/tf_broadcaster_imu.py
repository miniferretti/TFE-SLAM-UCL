#!/usr/bin/env python3
import rospy
import tf_conversions
import tf2_ros
import math
import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import Temperature, Imu

n_samples = 20
n = 0
calib = False
x, y, z, w = [0, 0, 0, 0]


def handle_imu_pose(msg):
    global n_samples, n, calib, x, y, z, w

    if calib == False:
        x += msg.orientation.x
        y += msg.orientation.y
        z += msg.orientation.z
        w += msg.orientation.w
        n += 1
        if n == n_samples + 1:
            calib = True
    else:

        the_norm = math.sqrt((msg.orientation.x - x/n_samples)**2 + (msg.orientation.y - y/n_samples)
                             ** 2 + (msg.orientation.z - z/n_samples)**2 + (msg.orientation.w - w/n_samples)**2)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "base_imu"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 1.1
        t.transform.rotation.x = (msg.orientation.x - x/n_samples) / the_norm
        t.transform.rotation.y = (msg.orientation.y - y/n_samples) / the_norm
        t.transform.rotation.z = (msg.orientation.z - z/n_samples) / the_norm
        t.transform.rotation.w = (msg.orientation.w - w/n_samples) / the_norm
        print(t.transform.rotation)

        br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu')
    rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
    rospy.spin()
