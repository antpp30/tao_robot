#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
 
def talker():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('Talker_Test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    speed = 10.0
    turn = 5.0
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0.0; twist.linear.z = 0.0;
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = turn
        pub.publish(twist)
        print (twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
