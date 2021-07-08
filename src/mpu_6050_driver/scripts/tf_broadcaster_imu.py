#!/usr/bin/env python3

import rospy
import tf_conversions
from tf.transformations import *
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

    if calib == True:
        x += msg.orientation.x
        y += msg.orientation.y
        z += msg.orientation.z
        w += msg.orientation.w
        n += 1
        if n == n_samples + 1:
            calib = True
    else:

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "base_imu"
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 1.1

        the_norm = math.sqrt((msg.orientation.x - x/n_samples)**2 + (msg.orientation.y - y/n_samples)
                             ** 2 + (msg.orientation.z - z/n_samples)**2 + (msg.orientation.w - w/n_samples)**2)

        q_rot = quaternion_from_euler(0, math.pi, 0)

        q_or = [(msg.orientation.x - x/n_samples) / the_norm, (msg.orientation.y - y/n_samples) / the_norm,
                (msg.orientation.z - z/n_samples) / the_norm, (msg.orientation.w - w/n_samples) / the_norm]

        q_f = quaternion_multiply(q_rot, q_or)

        t.transform.rotation = geometry_msgs.msg.Quaternion(
            q_f[0], q_f[1], q_f[2], q_f[3])

        br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu')
    rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
    rospy.spin()
