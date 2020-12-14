#!/usr/bin/env python 
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
import record
import googletranslate

# 节点初始化 
rospy.init_node('move_test', anonymous=True)  
  
# 订阅move_base服务器的消息  
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

rospy.loginfo("Waiting for move_base action server...")  

# 等待连接服务器，5s等待时间限制 
while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
    rospy.loginfo("Connected to move base server")  

roomList = [[0.25, -9.5,0.000],[3.2,-9.5,3.132],[-9.0,4.5,-1.650],[9.0,7.0,-1.650],[-9.5,6,1.650],[-3.5,9.5,-0.05]]

r = record.Recoder()
r.recoder()
r.savewav("/home/zm/Desktop/test.wav")
t = googletranslate.Translate()
t.match()
roomNumber = t.getRealRoomNumber()
target = None
if roomNumber is not None:
    target = Pose(Point(roomList[roomNumber][0],roomList[roomNumber][1],roomList[roomNumber][2]), Quaternion(0.000, 0.000, 0.645, 0.764))  
else:
    target = Pose(Point(0.25, -9.5, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764))  
# 设定目标点  

goal = MoveBaseGoal()  
goal.target_pose.pose = target  
goal.target_pose.header.frame_id = 'map'  
goal.target_pose.header.stamp = rospy.Time.now()  

rospy.loginfo("Going to: " + str(target))  

# 向目标进发  
move_base.send_goal(goal)  

# 五分钟时间限制  
finished_within_time = move_base.wait_for_result(rospy.Duration(300))   

# 查看是否成功到达  
if not finished_within_time:  
    move_base.cancel_goal()  
    rospy.loginfo("Timed out achieving goal")  
else:  
    state = move_base.get_state()  
    if state == GoalStatus.SUCCEEDED:  
        rospy.loginfo("Goal succeeded!")
    else:  
      rospy.loginfo("Goal failed！ ")  

