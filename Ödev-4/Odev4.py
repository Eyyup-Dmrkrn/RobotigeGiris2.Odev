#!/usr/bin/env python

import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt, pi
import numpy
import json

class turtle():

    def __init__(self):

        rospy.init_node('turtlebot_controller', anonymous=True)
        self.vel_msg = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

        f = open('veriler.json')
        self.veriler = json.load(f)

        self.MAX_VEL =  self.veriler ['MAX_VEL']
        self.MAX_ANGL_VEL =  self.veriler ['MAX_ANGL_VEL']
        self.MAX_FOLLOW_DISTANCE =  self.veriler ['MAX_FOLLOW_DISTANCE']

        self.FRAME_LOW_BODER = 0.5
        self.FRAME_HIGH_BORDER = 10.5

        self.desiredAngle = pi/6

    def callback(self, data):
        self.pose = data
 
    def wallControl(self, vel_msg):

        if self.pose.x > self.FRAME_LOW_BODER and self.pose.x < self.FRAME_HIGH_BORDER  and self.pose.y > self.FRAME_LOW_BODER and self.pose.y < self.FRAME_HIGH_BORDER:
            self.vel_msg.linear.x = self.maks_hiz / 2
            self.vel_msg.publish(self.vel_msg) 

        else:                 
            if self.pose.theta > 0 and (self.pose.x > self.FRAME_HIGH_BORDER or self.pose.x < self.FRAME_LOW_BODER):
                self.desiredAngle = pi - self.pose.theta
                self.rotate(self.desiredAngle)
                self.vel_msg.publish(self.vel_msg)

            elif (self.pose.theta > 0 and self.pose.y > self.FRAME_HIGH_BORDER) or (self.pose.theta < 0 and self.pose.y < self.FRAME_LOW_BODER):
                self.desiredAngle = -self.pose.theta
                self.rotate(self.desiredAngle)
                self.vel_msg.publish(self.vel_msg)

            elif self.pose.theta < 0 and (self.pose.x > self.FRAME_HIGH_BORDER or self.pose.x < self.FRAME_LOW_BODER):
                self.desiredAngle = -(pi + self.pose.theta)
                self.rotate(self.desiredAngle)
                self.vel_msg.publish(self.vel_msg)
            else:
                break

    def move(self):

        while pose is None:
            rospy.sleep(0.01)

        vel_msg = Twist()

        while True:
            self.wallControl(vel_msg)
            self.rate.sleep()

    def rotate(self, desiredAngle):

        while self.pose is None:
            rospy.sleep(0.01)

        while True:

            diff = abs(self.pose.theta - desiredAngle)
            print ('Angle Diff: ', diff)

            if diff > 0.2:
                self.vel_msg.angular.z = self.MAX_VEL
                self.vel_msg.publish(self.vel_msg)
            else:
                self.vel_msg.angular.z = 0
                self.vel_msg.publish(self.vel_msg)
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        #Testing our function
        turtle = turtle()
        turtle.rotate(pi/8)
        while True:
            turtle.move()

    except rospy.ROSInterruptException: 
        pass
