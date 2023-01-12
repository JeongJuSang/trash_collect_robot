#! /usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import RecoveryStatus
from std_srvs.srv import Empty

class RAINBOW():
    def __init__(self):
        rospy.init_node('scout_nav', anonymous=False)
        self.goal = MoveBaseGoal()
        self.i = 0
        self.flag = True
        self.flag_red = False
        self.status_msg = "empty"
        self.rcvy_status_msg = -1
        #self.rate = rospy.Rate(0.25)


        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.mb_status_sub = rospy.Subscriber("/move_base/result",MoveBaseActionResult,self.mb_CB)
        self.rcvy_status_sub = rospy.Subscriber("/move_base/recovery_status", RecoveryStatus, self.rcvy_CB)
        rospy.Timer(rospy.Duration(1),callback = self.renew_goal)
        rospy.spin()

    def mb_CB(self,data):
        # try:
        #     self.status_msg = data.status.text
        # except:
        #     self.status_msg = "empty"
        self.status_msg = data.status.text

    def rcvy_CB(self,data):
        self.rcvy_status_msg = data.current_recovery_number

    
    def renew_goal(self,event):
        # lst = ["red", "orange", "yellow", "green", "blue", "navy", "purple", "white"]
        # lst = ["red", "orange", "yellow", "green", "blue"]
        lst = ["red", "orange", "yellow", "blue"]

        #self.rate.sleep()
        if self.status_msg == "Goal reached.":
            self.flag = True
            # self.i = (self.i + 1) % 8
            # self.i = (self.i + 1) % 6
            self.i = (self.i + 1) % 5

            rospy.loginfo("---------------------------")
            rospy.loginfo("----- Reached at Goal -----")
            rospy.loginfo("---------------------------")
            self.status_msg = "empty_red"

        elif self.rcvy_status_msg == 1:
            self.flag = True
            rospy.loginfo("-----------------------------")
            rospy.loginfo(" Cancle Rotate & Resend Goal ")
            rospy.loginfo("-----------------------------")
            self.rcvy_status_msg = -1
        
        elif self.status_msg == "Failed to find a valid plan. Even after executing recovery behaviors.":
            self.flag = True
            rospy.loginfo("---------------------------")
            rospy.loginfo("------- Resend Goal -------")
            rospy.loginfo("---------------------------")
            self.status_msg = "empty"
        
        self.move_to(lst[self.i])

        #rospy.loginfo("Move to {}".format(i))
        
                

    def move_to(self, color):
        
        if color == "red":
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position = Point(-6.668271541595459,3.472092628479004,0)
            self.goal.target_pose.pose.orientation = Quaternion(0,0,0.9999867465737107,0.0051484635499707216)
            # rospy.ServiceProxy("request_nomotion_update",Empty)()

            
        elif color == "orange":
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position = Point(-2.176192045211792, -6.189762115478516, 0)
            self.goal.target_pose.pose.orientation = Quaternion(0,0,-0.016358493375350534, 0.999866190894806)
            # rospy.ServiceProxy("request_nomotion_update",Empty)()


            if self.status_msg == "empty_red":
                self.flag_red = True      
            
        elif color == "yellow":
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position = Point(0.1949320137500763, 0.03548920527100563, 0)
            self.goal.target_pose.pose.orientation = Quaternion(0,0,0.01352914218294353, 0.9999084769676642)

            if self.status_msg == "empty_red":
                self.flag_red = True 


        # elif color == "green":
        #     self.goal.target_pose.header.frame_id = 'map'
        #     self.goal.target_pose.header.stamp = rospy.Time.now()
        #     self.goal.target_pose.pose.position = Point(15.2774734497, 0.408093512058 ,0)
        #     self.goal.target_pose.pose.orientation = Quaternion(0,0,-0.456903844298, 0.889516091516)



        elif color == "blue":
            self.status_msg = "empty"
            self.flag == False
                     

        if self.flag_red == True:
            time_past = rospy.Time.now().to_sec()
            time_now = rospy.Time.now().to_sec()

            while (time_now - time_past) < 3.0:
                # rospy.ServiceProxy("request_nomotion_update",Empty)()
                # rospy.loginfo("Request_nomotion_update...")
                time_now = rospy.Time.now().to_sec()
                rospy.sleep(0.3)
                rospy.ServiceProxy('move_base/clear_costmaps',Empty)()
                rospy.loginfo("Costmaps are cleared...")
            
            self.flag_red = False
            self.status_msg = "empty"

        if self.flag == True:
            rospy.ServiceProxy('move_base/clear_costmaps',Empty)()
            rospy.loginfo("Costmaps are cleared...")

            self.ac.send_goal(self.goal)
            rospy.loginfo("Send new goal...")
            self.flag = False               


        rospy.loginfo("Move to {}".format(color))

if __name__ =="__main__":
    RAINBOW()