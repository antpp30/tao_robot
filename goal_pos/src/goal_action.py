#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import time
def movebase_client():  # Lab_1
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo('got server')
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"         #map,odom
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 10.11 
    goal.target_pose.pose.position.y = -7.44
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.99
    goal.target_pose.pose.orientation.w = 0.03
    client.send_goal(goal)
    rospy.loginfo('sent goal')
    rospy.loginfo(goal)
    client.wait_for_result()
    return client.get_result()

def movebase_client_2(): #Lab_3
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo('got server')
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"         #map,odom
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 21.34
    goal.target_pose.pose.position.y = -9.62
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.99
    goal.target_pose.pose.orientation.w = 0.10
    client.send_goal(goal)
    rospy.loginfo('sent goal')
    rospy.loginfo(goal)
    client.wait_for_result()
    return client.get_result()

def movebase_client_3(): #Hall of innovation
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo('got server')
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"         #map,odom
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 7.85
    goal.target_pose.pose.position.y = 5.33
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.99
    goal.target_pose.pose.orientation.w = -0.02
    client.send_goal(goal)
    rospy.loginfo('sent goal')
    rospy.loginfo(goal)
    client.wait_for_result()
    return client.get_result()

def movebase_client_4(): #HOME
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo('got server')
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"         #map,odom
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 0.99
    client.send_goal(goal)
    rospy.loginfo('sent goal')
    rospy.loginfo(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        rospy.init_node('simple_nav_goal')

        # result = movebase_client()
        # print "goal IN"

        # result_2 = movebase_client_2()
        # print result_2

        result_3 = movebase_client_3()
        print result_3

        

        result_4 = movebase_client_4()
        print result_4

    except rospy.ROSInterruptException:
        print "interrupted"
