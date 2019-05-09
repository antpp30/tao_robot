#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from math import sin, cos, pi
import tf
import time



# Recieve VL,VR
class Serial_recieve():
    def __init__(self, port, buadrate):
        rospy.init_node("listen_Vel", Float64MultiArray, queue_size=10)

        self.serial = serial.Serial(port = port, buadrate = buadrate)
        pass

class OdometryV():
    def __init__(self, V_L, V_R):
        
        # Recieve wheel's velocity
        self.VR = V_R
        self.VL = V_L
        print(self.VL, self.VR)
        # Initial position of robot
        self.x = 0
        self.y = 0
        self.th = 0

        # Initial robot's velocity
        self.V_rx = 0
        self.V_ry = 0
        self.W_r = 0

        # Distance form center to wheel
        self.L = 0.5

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        # Create object of tf
        self.odom_broadcaster = tf.TransformBroadcaster()

    def cal_odometry(self):
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        self.last_time = self.current_time

        self.V_rx = (self.VR + self.VL)/2
        self.V_ry = 0
        self.W_r = (self.VR - self.VL)/self.L

        self.th += self.W_r * dt

        delta_x = (self.V_rx * cos(self.th)) * dt
        delta_y = (self.V_rx * sin(self.th)) * dt

        self.x += delta_x
        self.y += delta_y

        # Create quaternion from yaw
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform((self.x, self.y, 0), self.odom_quat, 
                                            self.current_time, 
                                            "base_link", "odom")
        
        
    def publish_odom(self):
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0), Quaternion(*self.odom_quat))

        # set the velocity
        odom.twist.twist = Twist(Vector3(self.V_rx, self.V_ry, 0), Vector3(0, 0, self.W_r))
        print(odom.pose.pose)
        # print(odom.twist.twist)

        # publish the message
        self.odom_pub.publish(odom)


if __name__ == '__main__':

    
    rospy.init_node('odometry_publisher')
    rate = rospy.Rate(10)

    # Declare object
    # serial = Serial_recieve('/dev/ttySTM32',115200)

    odom = OdometryV(1.0, 0)
    
    while (not rospy.is_shutdown()):
        rate.sleep()

        odom.cal_odometry()
        odom.publish_odom()
        # time.sleep(1)
        