#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
import numpy as np
from math import radians, pi

global target
global pub
target = 355
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

def listener():
    
    rospy.init_node('target', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()

def callback(data):
    quat_raw = data.pose.pose.orientation
    quat_arr = np.array([quat_raw.x, quat_raw.y, quat_raw.z, quat_raw.w])
    yaw_rad = tf.transformations.euler_from_quaternion(quat_arr, 'sxyz')[2]
    degree = (yaw_rad * 180)/pi

    W = 0.75
    if (degree >= target):
        if (degree > 180)
            degree = ()
        W = 0.0
    # V = 0.1 * (target - pos_x)
    print(degree, W)

    rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = W
    pub.publish(twist)
    rate.sleep

if __name__ == '__main__':
    listener()