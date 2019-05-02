#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import time


class turtlebot(object):

	def __init__(self):
		rospy.init_node('distance')
		self.dist = LaserScan()
		self.rate = rospy.Rate(10)
		self.sub = rospy.Subscriber("/laserscan", LaserScan, self.distance)
		self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 10)


	def move(self):
		msg = Twist()
			
		while self.dist.ranges[720] > 1.5 :
			print self.dist.ranges[720]
			msg.linear.x = 0.5
			msg.linear.y = 0
			self.pub.publish(msg)
			self.rate.sleep()
		
		# stop moving
		msg.linear.x = 0
		msg.linear.y = 0
		self.pub.publish(msg)
		self.rate.sleep()


	def distance(self, distance):
		self.dist = distance


if __name__ == '__main__':
	x = turtlebot()
	x.rate.sleep()
	x.move()