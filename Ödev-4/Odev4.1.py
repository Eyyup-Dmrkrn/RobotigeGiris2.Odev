#!/usr/bin/env python

import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow, atan2,sqrt, pi
import json

class turtle():

    def __init__(self):

        rospy.init_node('firstRobot', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

        f = open('/home/eyyup/catkin_ws/src/ikinci_paket/node/veriler.json')
        self.veriler = json.load(f)

        self.MAX_VEL =  self.veriler ['MAX_VEL']
        self.MAX_ANGL_VEL =  self.veriler ['MAX_ANGL_VEL']
        self.MAX_FOLLOW_DISTANCE =  self.veriler ['MAX_FOLLOW_DISTANCE']

        self.FRAME_LOW_BORDER = 0.8
        self.FRAME_HIGH_BORDER = 10.3

        self.desiredAngle = 0

        self.vel_msg = Twist()

    def callback(self, data):
        self.pose = data
 
    def wallErrorControl(self, vel_msg):
                
        if self.pose.theta > 0 and (self.pose.x > self.FRAME_HIGH_BORDER or self.pose.x < self.FRAME_LOW_BORDER):
            self.desiredAngle = pi - self.pose.theta
            self.rotate(pi - self.pose.theta)
            self.velocity_publisher.publish(self.vel_msg)

        elif (self.pose.theta > 0 and self.pose.y > self.FRAME_LOW_BORDER): 
            self.desiredAngle = -self.pose.theta
            self.rotate(self.desiredAngle)
            self.velocity_publisher.publish(self.vel_msg)

        elif self.pose.theta < 0 and self.pose.y < self.FRAME_LOW_BORDER:
            self.desiredAngle = -self.pose.theta - pi/2
            self.rotate(self.desiredAngle)
            self.velocity_publisher.publish(self.vel_msg)

        elif self.pose.theta < 0 and (self.pose.x > self.FRAME_HIGH_BORDER or self.pose.x < self.FRAME_LOW_BORDER):
            self.desiredAngle = -(pi + self.pose.theta)
            self.rotate(self.desiredAngle)
            self.velocity_publisher.publish(self.vel_msg)    

        elif self.pose.theta < 0 and self.pose.x < self.FRAME_LOW_BORDER:
            self.desiredAngle = -pi - self.theta
            self.rotate(self.desiredAngle)
            self.velocity_publisher.publish(self.vel_msg)       
        
        elif self.pose.theta > 0 and self.pose.y > self.FRAME_HIGH_BORDER:
            self.desiredAngle = - self.theta - pi/2
            self.rotate(self.desiredAngle)
            self.velocity_publisher.publish(self.vel_msg)
    
        elif self.pose.theta < 0 and self.pose.y < self.FRAME_LOW_BORDER:
            self.desiredAngle =  pi/2 - self.theta 
            self.rotate(self.desiredAngle)
            self.velocity_publisher.publish(self.vel_msg)
        
                
    def move(self):

        while self.pose is None:
            rospy.sleep(0.01)

        while True:            
            if self.pose.x > self.FRAME_LOW_BORDER and self.pose.x < self.FRAME_HIGH_BORDER  and self.pose.y > self.FRAME_LOW_BORDER and self.pose.y < self.FRAME_HIGH_BORDER:
                    self.vel_msg.linear.x = self.MAX_VEL / 4
                    self.velocity_publisher.publish(self.vel_msg) 
            else:                 
                self.wallErrorControl(self.vel_msg)    

            self.rate.sleep()

    def rotate(self, desiredAngle):

        while self.pose is None:
            rospy.sleep(0.01)
            
        while True:

            diff = abs(self.pose.theta - desiredAngle)
            print ('Angle Diff: ', diff)

            if diff > 0.3:
                self.vel_msg.angular.z = self.MAX_ANGL_VEL 
                self.velocity_publisher.publish(self.vel_msg)
            else:
                self.vel_msg.angular.z = 0
                self.velocity_publisher.publish(self.vel_msg)
                break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        turtle = turtle()
        turtle.rotate(pi/9)
        turtle.move()

    except rospy.ROSInterruptException: 
        pass
