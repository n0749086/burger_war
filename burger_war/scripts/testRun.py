#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import json


class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        # camera subscribver
        # please uncoment out if you use camera
        # for convert image topic to opencv obj
        self.img = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)

        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # war status
        self.war_state = rospy.Subscriber("war_state", String, self.stateCallback)

        self.lx = 0
        self.ly = 0
        self.lz = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
        self.SPEED = 0.2
        self.ANGLE = 1

    # lidar scan topic call back sample
    # update lidar scan state
    def lidarCallback(self, data):
        self.scan = data
        #rospy.loginfo(self.scan)

   # camera image call back sample
    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def isNearWall(self, scan):
        if not (len(scan) == 360):
            return 0
        forword_scan = scan[:10] + scan[-10:]
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return -1
        back_scan = scan[175:185]
        back_scan = [x for x in back_scan if x > 0.1]
        if min(back_scan) < 0.2:
            return 1
        return 0

    def randomWalk(self):
        value = random.randint(1,1000)
        if value < 250:
            self.lx = self.SPEED
            self.az = 0
        elif value < 500:
            self.lx = -self.SPEED
            self.az = 0
        elif value < 750:
            self.lx = 0
            self.az = self.ANGLE
        elif value < 1000:
            self.lx = 0
            self.az = -self.ANGLE
        else:
            self.lx = 0
            self.az = 0

    def calcTwist(self):
        # ぶつかりそうなら、後退
        val = self.isNearWall(self.scan.ranges)
        if val != 0:
            self.lx = val * 2 * self.SPEED
            self.az = val * self.ANGLE
        else:
            # とりあえず、ランダムウォーク
            self.randomWalk()

        twist = Twist()

        twist.linear.x = self.lx
        twist.linear.y = self.ly
        twist.linear.z = self.lz
        twist.angular.x = self.ax
        twist.angular.y = self.ay
        twist.angular.z = self.az

        return twist

    def stateCallback(self, state):
        dic = json.loads(state.data)
        #rospy.loginfo(dic)
        #rospy.loginfo(("score", dic["scores"]["r"]))

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()

