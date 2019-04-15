#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Quaternion
from math import atan2

class turtlebot(object):

	def __init__(self):
		rospy.init_node('location')

		self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size = 10)
		self.sub = rospy.Subscriber("/odom", Odometry, self.new_pos)

		self.pose = Point()
		self.orientation = Quaternion()
		self.rate = rospy.Rate(10)

	def new_pos(self, msg):
		self.pose = msg.pose.pose.position
		self.orientation = msg.pose.pose.orientation

	def move(self, goal_pose):
		
		msg = Twist()

		# find the distance of the goal
		distance = Point()
		distance.x = abs(goal_pose.x - self.pose.x)
		distance.y = abs(goal_pose.y - self.pose.y)
		while (distance.x > 0.1) or (distance.y > 0.1) : # can be done with euclidean distance

			# find the angle to turn
			(notx, noty, theta) = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
			goal_ang = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

			# face the goal point
			while abs(goal_ang - theta) > 0.1 :
				msg.linear.x = 0
				msg.linear.y = 0
				msg.angular.z = 0.1
				self.pub.publish(msg)

				self.rate.sleep()
				(notx, noty, theta) = euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
				goal_ang = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

			# stop turning
			msg.linear.x = 0
			msg.linear.y = 0
			msg.angular.z = 0.0
			self.pub.publish(msg)

			# go straight 
			msg.linear.x = 0.5
			msg.linear.y = 0
			msg.angular.z = 0
			self.pub.publish(msg)
			self.rate.sleep()
			distance.x = abs(goal_pose.x - self.pose.x)
			distance.y = abs(goal_pose.y - self.pose.y)

			# print self.pose.x
			# print self.pose.y

			# print distance.x
			# print distance.y

		# stop moving
		msg.linear.x = 0
		msg.linear.y = 0
		msg.angular.z = 0.0
		self.pub.publish(msg)


if __name__ == '__main__':
	x = turtlebot()
	goal_pose = Point()
	goal_pose.x = input("Set your x goal: ")
	goal_pose.y = input("Set your y goal: ")
	x.move(goal_pose)
