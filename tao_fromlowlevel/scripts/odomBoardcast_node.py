#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Float32MultiArray, String
from math import sin, cos, pi
import tf
import time
from decimal import Decimal

# Recieve VL,VR
class Serial_recieve():
    def __init__(self):
        # print("Serial_init")
        rospy.Subscriber("chatter", Float32MultiArray, self.getVel)
        self.V_L = 0
        self.V_R = 0
        

    def getVel(self, data):
        self.V_L = data.data[0]
        self.V_R = data.data[1]
        # print(self.V_L, self.V_R)

        # Echo topic for Wheel's velocity from Encoder
        V_wheel = Float32MultiArray(data = [data.data[0], data.data[1]])
        V_pub = rospy.Publisher('getvel', Float32MultiArray, queue_size=20)
        V_pub.publish(V_wheel)
        
class OdometryV():
    def __init__(self):
        # print("Odom_init")

        # Create object of serial_recieve class
        self.object = Serial_recieve()

        # Recieve wheel's velocity
        self.VR = 0
        self.VL = 0

        # Initial position of robot
        self.x = 0
        self.y = 0
        self.th = 0

        # Initial robot's velocity
        self.V_rx = 0
        self.V_ry = 0
        self.W_r = 0

        # Distance form wheel to wheel
        self.L = 0.39

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Create object of tf
        self.odom_broadcaster = tf.TransformBroadcaster()

    def cal_odometry(self):
        self.VL = round(self.object.V_L, 3)
        self.VR = round(self.object.V_R, 3)
        # print(self.VL, self.VR)

        V_wheel_round = Float32MultiArray(data = [self.VL, self.VR])
        Vn_pub = rospy.Publisher('getvelround', Float32MultiArray, queue_size=20)
        Vn_pub.publish(V_wheel_round)

        # VL_pub = rospy.Publisher('getVL',String, queue_size=10)
        # VR_pub = rospy.Publisher('getVR',String, queue_size=10)
        # VL_pub.publish(str(self.VL))
        # VR_pub.publish(str(self.VR))

        # print(self.VL, self.VR)
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()

        # Robot's velocity 
        self.V_rx = (self.VR + self.VL)/2
        self.V_ry = 0
        self.W_r = (self.VR - self.VL)/self.L

        self.th += self.W_r * dt

        delta_x = (self.V_rx * cos(self.th)) * dt
        delta_y = (self.V_rx * sin(self.th)) * dt

        self.x += delta_x
        self.y += delta_y
        
        # Create quaternion from yaw
        self.odom_quat = tf.transformations.quaternion_from_euler(
            0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform((self.x, self.y, 0), self.odom_quat,
                                            self.current_time,
                                            "base_link", "odom")
        # self.odom_broadcaster.sendTransform((0.16319, 0.0, 0.1855), tf.transformations.quaternion_from_euler(0, 0, 0), 
                                            # self.current_time, 
                                            # "laser_frame", "base_link")

    def publish_odom(self):
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0),
                              Quaternion(*self.odom_quat))

        # set the velocity
        odom.twist.twist = Twist(
            Vector3(self.V_rx, self.V_ry, 0), Vector3(0, 0, self.W_r))
        # print(odom.pose.pose)
        # print(odom.twist.twist)

        # publish the message
        self.odom_pub.publish(odom)
        self.last_time = self.current_time

if __name__ == '__main__':

    rospy.init_node('odometry_boardcast_node', anonymous=False)
    rate = rospy.Rate(20)
    # print("Start")    

    odom = OdometryV()
    ser = Serial_recieve()

    while (not rospy.is_shutdown()):
        odom.cal_odometry()
        odom.publish_odom()
        rate.sleep()
        # time.sleep(1)
