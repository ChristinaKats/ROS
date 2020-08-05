#!/usr/bin/env python

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan


class GoForward(): # go straight until you reach an odstacle 
# The goal is to find a random wall so later the turtlebot can follow it

	def __init__(self):

		print 'Started'
		rospy.on_shutdown(self.shutdown)

		rospy.init_node('GoForward', anonymous=False)

		# topic for moving turtlebot
		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
		
		self.infront_distance = 0 # distance of the obstacle in front of the turtlebot
 
		r = rospy.Rate(10)

		# move turtlebot
		move_cmd = Twist()
		# move_cmd.linear.x = 0.5 # go forward at 0.5 m/s
		# move_cmd.angular.z = 0 # go straight only


		while not rospy.is_shutdown(): # Ctr + c
			if self.check_next_pos_scan(): # free space in front of the turtlebot
				move_cmd.linear.x = 0.5 # go forward at 0.5 m/s
				move_cmd.angular.z = 0 # go straight only
				self.cmd_vel.publish(move_cmd)
			else: # obstacle ahead
				move_cmd.linear.x = 0 # stop moving
				move_cmd.angular.z = 0 # stop turning
				self.cmd_vel.publish(move_cmd)

			r.sleep()

                        
	def shutdown(self): # stop turtlebot
		rospy.loginfo("Stop TurtleBot")

		move_cmd = Twist()
		move_cmd.linear.x = 0 # stop moving
		move_cmd.angular.z = 0 # stop turning

		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)


	def check_next_pos_scan(self):
		# rospy.loginfo("Check next position")

		# topic for the laserScans og kinect
		distance_infront = rospy.Subscriber('/scan', LaserScan, self.distance_from_obstacle_ahead)

		# rospy.loginfo(self.infront_distance)

		# If there is an odstacle 2.5m ahead, you should stop moving
		if math.isnan(self.infront_distance) == False and self.infront_distance < 2.5: 
			return False
		else: # else, free space continue straight
			return True


	def distance_from_obstacle_ahead(self, data): # find my current position
		self.infront_distance = data.ranges[320]


	def get_rotation(self, msg): # find where I am looking at
		global roll, pitch, yaw
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		# yaw euler
		self.degrees = (yaw)/(2 * math.pi) * 360 # degrees
		# print degrees

 
if __name__ == '__main__':
    try:
        GoForward()
    except:
        rospy.loginfo("GoForward node terminated.")


