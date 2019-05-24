#!/usr/bin/env python

import sys
from PyQt4 import uic, QtGui
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray

class MyWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        uic.loadUi('/home/thanachot/ttar/src/goal_pos/ui/Tao.ui', self)
    
        self.office_btn.clicked.connect(self.office_path)
        self.lab1_btn.clicked.connect(self.lab1_path)
        self.lab3_btn.clicked.connect(self.lab3_path)
        # ROS
        rospy.init_node('simple_nav_goal')
        rospy.Subscriber("chatter", Float32MultiArray, self.get_IR)
        self.state_IR = 0

    # Check IR's state
    # state = 1 ==> non-detected
    # state = 0 ==> detected
    def get_IR(self, data):
        self.state_IR = data.data[2]
        check = Float32MultiArray(data = [self.state_IR])
        check_pub = rospy.Publisher('getState', Float32MultiArray, queue_size=20)
        check_pub.publish(check)
    
    def office_path(self):
        state = self.state_IR
        if(state == 1):
            rospy.logerr("Please insert your documents and choose again")

        elif(state == 0):
            print("Office")
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
            #return client.get_result()
            # Reget state
            while(True):
                state = self.state_IR
                if(state == 1):
                    break
            self.home()
            

    def lab1_path(self):
        state = self.state_IR
        if(state == 1):
            rospy.logerr("Please insert your documents and choose again")
        elif(state == 0):    
            rospy.logerr("Lab1")
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
            # client.wait_for_result()
            #return client.get_result()
            rospy.logerr("goal")
            while(True):
                state = self.state_IR
                if(state == 1):
                    break
            self.home()

    def lab3_path(self):
        state = self.state_IR
        if(state == 1):
            rospy.logerr("Please insert your documents and choose again")
        elif(state == 0):    
            rospy.logerr("Lab3")
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
            rospy.logerr("goal_3")
            while(True):
                state = self.state_IR
                if(state == 1):
                    break
            self.home()

    def home(self):
        print("home")
        rospy.sleep(3)
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
    # try:
    app = QtGui.QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
    
    

        

    # except rospy.ROSException as e:
    #     print(e)