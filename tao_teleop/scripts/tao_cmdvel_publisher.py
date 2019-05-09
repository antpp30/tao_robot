#!/usr/bin/env python
from __future__ import print_function
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w     
   a    s    d
        x    
---------------------------
Rules:
w/x : increase/decrese linear velocity by 0.1 m/s
a/d : increase/decrease angular velocity by 0.1 rad/s
s : stop 
CTRL-C to quitz
---------------------------
Have a fun!!!
"""
speedBindings={
        'w':(0.1, 0),
        'a':(0, .5),
        'd':(0, -.5),
        's':(0, 0),
        'x':(-0.1, 0)
}
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tlinear_vel %s\tangular_vel %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('My_teleop_twist_keyboard')
    rate = rospy.Rate(10)

    speed = 0
    turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in speedBindings.keys():
                if (key == 's'):
                    speed = 0.0
                    turn = 0.0

                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]

                if (0.1> speed >-0.1):
                    speed = 0.0
                elif (speed >= 0.5):
                    speed = 0.5
                elif (speed <= -0.5):
                    speed = -0.5

                if (0.5> turn >-0.5):
                    turn = 0.0
                elif (turn >= 2.5):
                    turn = 2.5
                elif (turn <= -2.5):
                    turn = -2.5
                print ('------------------------------------------') 
                print(vels(speed,turn))

            elif (key == '\x03'):
                print ("Good bye")
                break

            twist = Twist()
            twist.linear.x = speed; twist.linear.y = 0.0; twist.linear.z = 0.0;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = turn
            pub.publish(twist)
            print (twist)
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)