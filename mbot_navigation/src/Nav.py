#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, PoseArray, Twist)
from sensor_msgs.msg import LaserScan

import numpy as np
import copy
import math
import Node
import A_Star
import record
import googletranslate
import time

class Nav:
    def __init__(self):
	self.robYaw = 0
        self.robX = 0
        self.robY = 0
	self.amcl_robYaw = 0
        self.amcl_robX = 0
        self.amcl_robY = 0
	self.twist = Twist()
	self.rate = rospy.Rate(8)
	self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
	self.laser_subscriber = rospy.Subscriber("/scan", LaserScan,
                                                  self.laser_callback,
                                                  queue_size=1)
        self.ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)

        self.amcl_pose_subcvriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback,
                                                     queue_size=1)

	self.odometry_subscriber = rospy.Subscriber("/odom", Odometry,
                                                     self.odometry_callback,
                                                     queue_size=1)

        rospy.loginfo("Map received. %d X %d, %f px/m" %
                      (self.ocuccupancy_map.info.width, self.ocuccupancy_map.info.height,
                       self.ocuccupancy_map.info.resolution))
        self.a_star = A_Star.A_Star(self.ocuccupancy_map.info.height, self.ocuccupancy_map.info.width,
                                    self.ocuccupancy_map.data)
        self.room = [[454, 293], [455, 328], [180, 90], [136, 456], [165, 93], [82, 193]]
        self.roomNumber = None
        self.startX = None
	self.startY = None
        self.startPose = None
        self.endPose = None
        self.map = None
	pathMap = None
        self.pathMap = None
        self.rootNode = None
        self.buff = []
        self.currentPose = None
        self.child = None
        self.isFind = False
        self.path = []
	self.scan = None
	self.ranges = None

    def set_start(self, startX, startY, roomNmber):
	self.roomNumber = roomNmber
        self.startX, self.startY = self.a_star.getIndex(startX, startY)
        self.startPose = Node.MyPoint(self.startY,self.startX)
        self.endPose = Node.MyPoint(self.room[roomNmber][0], self.room[roomNmber][1])
        self.map = self.a_star.generateMap()
	pathMap = copy.deepcopy(self.map)
        pathMap[self.startPose.row][self.startPose.col] = 1
        self.pathMap = copy.deepcopy(pathMap)
        self.rootNode = Node.TreeNode(self.startPose)
        self.buff = []
        self.currentPose = copy.copy(self.rootNode)

    def amcl_pose_callback(self, amcl_pose):
	pass
        self.amcl_robYaw = self.getHeading(amcl_pose.pose.pose.orientation)
        self.amcl_robX = round(amcl_pose.pose.pose.position.x,2)
        self.amcl_robY = round(amcl_pose.pose.pose.position.y,2)
	# print(self.robX)
	# print(self.robY)

    def laser_callback(self, scan):
	self.scan = scan
	self.ranges = np.ma.masked_invalid(self.scan.ranges).filled(self.scan.range_max)
	#print(self.ranges[89:269])
	#print(self.ranges[119:239])
	#print(self.ranges[239:359])

    def odometry_callback(self, odom):
        self.robYaw = self.getHeading(odom.pose.pose.orientation)
        self.robX = round(odom.pose.pose.position.x,2)
        self.robY = round(odom.pose.pose.position.y,2)
	#print(self.robX)
	#print(self.robY)
	#print(self.robYaw)

    def plan(self):
        while not self.isFind:
            for i in range(8):
                self.child = Node.TreeNode(Node.TreeNode(0, 0))
                self.child.pose = copy.copy(self.currentPose.pose)
                if i == 0:
                    self.child.pose.row -= 1
                    self.child.pose.g += self.a_star.LINECOST
                elif i == 1:
                    self.child.pose.row += 1
                    self.child.pose.g += self.a_star.LINECOST
                elif i == 2:
                    self.child.pose.col -= 1
                    self.child.pose.g += self.a_star.LINECOST
                elif i == 3:
                    self.child.pose.col += 1
                    self.child.pose.g += self.a_star.LINECOST
                elif i == 4:
                    self.child.pose.row -= 1
                    self.child.pose.col -= 1
                    self.child.pose.g += self.a_star.ARCOST
                elif i == 5:
                    self.child.pose.row += 1
                    self.child.pose.col -= 1
                    self.child.pose.g += self.a_star.ARCOST
                elif i == 6:
                    self.child.pose.row -= 1
                    self.child.pose.col += 1
                    self.child.pose.g += self.a_star.ARCOST
                elif i == 7:
                    self.child.pose.row += 1
                    self.child.pose.col += 1
                    self.child.pose.g += self.a_star.ARCOST
                self.child.pose.h = self.a_star.getH(self.endPose, self.child.pose)
                self.child.pose.setF()
                if self.a_star.needAdd(self.child.pose, self.map, self.pathMap):
                    self.currentPose.child.append(self.child)
                    self.child.parent = self.currentPose
                    self.buff.append(self.child)
                    self.pathMap[self.child.pose.row][self.child.pose.col] = 1
                else:
                    self.child = None
            itMin = self.buff[0]
            for i in self.buff:
                itMin = i if itMin.pose.f > i.pose.f else itMin
            self.currentPose = itMin
            self.buff.remove(itMin)
            if self.currentPose.pose.row == self.endPose.row and self.currentPose.pose.col == self.endPose.col:
                self.isFind = True
                return True

            if len(self.buff) == 0:
                return False

    def getPath(self):

        isFind = self.plan()
        if isFind:
            print("Find end position")
            while self.currentPose:
                # print((currentPose.pose.x, currentPose.pose.y))
                tmpX, tmpY = self.a_star.getCoordinate(self.currentPose.pose.row, self.currentPose.pose.col)
                self.path.append([round(tmpX,2), round(tmpY,2)])
                self.currentPose = copy.copy(self.currentPose.parent)
            return self.path
        else:
            return "Not Find"

    def getHeading(self, q):
        yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                         q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
        return yaw


    def move(self, option):
	if option == -1:
	    path = self.getPath()
	else:
	    path = self.getPath()[::-1]
        for i in range(1, len(path)):
            aimX = path[i][1]
            aimY = path[i][0]
            aimYaw = math.atan2(aimY - self.robY, aimX - self.robX)
            while 1:
                if abs(aimYaw - self.robYaw) > 0.05:
                    self.twist.linear.x = 0
                    #print("jiaodu", abs(abs(aimYaw) - abs(self.robYaw)))
                    self.twist.angular.z = (aimYaw - self.robYaw) * 2
                    self.pub.publish(self.twist)
                else:
                    #print(self.robX - aimX)
                    #print(self.robY - aimY)
                    if abs(aimX - self.robX) > 0.05 or abs(aimY - self.robY)>0.05:
                        self.twist.angular.z = 0
                        self.twist.linear.x = math.sqrt(math.pow((aimX - self.robX), 2) + math.pow((aimY - self.robY), 2)) * 2
                        # print("sudu", self.twist.linear.x)
                        self.pub.publish(self.twist)        
                    else:
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.pub.publish(self.twist)
                        break
	    print("Distance traveled "+str(i+1)+"/"+str(len(path)))


if __name__ == '__main__':
    rospy.init_node("mbot_nav")
    con = True
    while con:
        op = raw_input("Whether to continue(y/n)")
        if op == 'y':
            con = True
            r = record.Recoder()
            r.recoder()
            r.savewav("/home/zm/Desktop/test.wav")
            t = googletranslate.Translate()
            t.match()
            roomNumber = t.getRealRoomNumber()
            if roomNumber is not None:
                nav = Nav()
                nav.set_start(-1, 1, roomNumber)
                nav.move(1)
                print("Arrived in room " + str(roomNumber + 1) + " !")
                time.sleep(10)
                navBack = Nav()
                navBack.set_start(-1, 1, roomNumber)
                navBack.move(-1)
                print("Returned to the starting point")
        elif op == 'n':
            con = False
    rospy.spin()
