#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

global target
global pub
target = 5.0
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
def listener():
    
    rospy.init_node('target', anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)
    rospy.spin()

def callback(data):
    pos_x = data.pose.pose.position.x
    V = 0.3
    if (pos_x >= target):
        V = 0.0
    # V = 0.1 * (target - pos_x)
    print(pos_x, V)

    rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = V
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub.publish(twist)
    rate.sleep

if __name__ == '__main__':
    listener()