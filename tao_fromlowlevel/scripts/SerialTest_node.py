#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
#from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension

#pub = rospy.Publisher('toggle_led', String, queue_size=10)
#serial_data = IntList()

dat = []

def talker(data):
    print(data.linear.x)
    print(data.angular.z)
    #global serial_data
    #serial_data = IntList()
    #serial_data.data = [data.linear.x, data.angular.z]

    #serial_data = data.linear.x 
    dat = Float64MultiArray(data=[data.linear.x, data.angular.z])


    pub = rospy.Publisher('toggle_led', Float64MultiArray, queue_size=10)
    rospy.init_node('listener', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #if rospy.is_shutdown():
    #rospy.loginfo(data.linear.x)
    pub.publish(dat)
    rate.sleep()

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('cmd_vel', Twist, talker)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

