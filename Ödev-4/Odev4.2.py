#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from math import pow,atan2,sqrt, pi, sin, cos
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import json

f = open('/home/eyyup/catkin_ws/src/ikinci_paket/node/veriler.json') 
veriler = json.load(f)

MAX_VEL =  veriler['MAX_VEL']
MAX_ANGL_VEL =  veriler['MAX_ANGL_VEL']
MAX_FOLLOW_DISTANCE = veriler['MAX_FOLLOW_DISTANCE']

Kv = 1
Kw = 0.7

pose = Pose()

def callback(msg):
	global pose
	pose=msg

def followCallback(robotpose):
	vel_msg = Twist()

	while True:    
		robotX = robotpose.x
		robotY = robotpose.y 
		distance = abs((sqrt((robotX-pose.x)**2 + (robotY-pose.y)**2)))

		if (distance < MAX_FOLLOW_DISTANCE): 
			break

		else:   
			v = Kv*distance
			if v > MAX_VEL: 
				v = MAX_VEL

			desiredHeading = atan2(robotY-pose.y, robotX-pose.x) 
			e_phi = atan2(sin(desiredHeading-pose.theta), cos(desiredHeading-pose.theta))
			w = Kw*e_phi

			if w > MAX_ANGL_VEL: 
				w = MAX_ANGL_VEL

			vel_msg.linear.x = v
			vel_msg.angular.z = w 

			vel_pub.publish(vel_msg) 

if __name__ == '__main__':

	rospy.init_node('secondRobot', anonymous = True)
	rospy.Subscriber('/turtle2/pose', Pose, callback)
	vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('/turtle1/pose', Pose, followCallback)

	loop_rate = rospy.Rate(10)
	
	rospy.spin()

#NOT: Follow trajectory için integral problem çıktığından PI yerine P controller kullandım.