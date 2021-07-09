#!/usr/bin/env python3

import rospy
import tf_conversions
from tf.transformations import *
import tf2_ros
import math
import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import Temperature, Imu

n_samples = 200
n = 0
calib = False
x, y, z = [[], [], []]
rot = None


def average(lst):
    return sum(lst)/len(lst)


def handle_imu_pose(msg):
    global n_samples, n, calib, x, y, z, w, rot

    if n < n_samples:
        a = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        x.append(a[0])
        y.append(a[1])
        z.append(a[2])
        n += 1

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "base_imu"
    t.transform.translation.x = 0.045
    t.transform.translation.y = 0
    t.transform.translation.z = 0.055

    q_rot = quaternion_from_euler(
        0-average(x), -(math.pi/2)-average(y), 0-average(z))  # Correction of the initial offset of the values of the IMU

    q_or = [msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w]

    q_f = quaternion_multiply(q_rot, q_or)

    t.transform.rotation = geometry_msgs.msg.Quaternion(
        q_f[0], q_f[1], q_f[2], q_f[3])

    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster_imu')
    rospy.Subscriber('/imu/data', Imu, handle_imu_pose)
    rospy.spin()
