#!/usr/bin/env python

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class GoForward(): # go straight until you reach an odstacl

	def __init__(self):

		print 'Started'
		rospy.on_shutdown(self.shutdown)

		rospy.init_node('GoForward', anonymous=False)

		self.cmd_vel = rospy.Publisher('/robot1/cmd_vel_mux/input/navi', Twist, queue_size=10)
		
		# curr_pos_sub = rospy.Subscriber('/robot1/move_base/local_costmap/costmap', OccupancyGrid, self.my_position)
		self.x = 0
		self.y = 0

		self.degrees = 0

		self.map_data = numpy.zeros(shape=(400,400))

		r = rospy.Rate(10);

		# move turtlebot
		move_cmd = Twist()
		move_cmd.linear.x = 0.8 # go forward at 0.5 m/s
		move_cmd.angular.z = 0 # go straight only


		while not rospy.is_shutdown(): # Ctr + c
			if self.check_next_pos():
				self.cmd_vel.publish(move_cmd)
			else:
				move_cmd.linear.x = 0 # stop moving
				move_cmd.angular.z = 0 # go straight only
				self.cmd_vel.publish(move_cmd)

			r.sleep()

                        
	def shutdown(self): # stop turtlebot
		rospy.loginfo("Stop TurtleBot")

		move_cmd = Twist()
		move_cmd.linear.x = 0 # stop moving
		move_cmd.angular.z = 0 # go straight only

		self.cmd_vel.publish(move_cmd)
		rospy.sleep(1)

	def check_next_pos(self):
		# rospy.loginfo("Check next position")

		# find my position
		curr_pos_sub = rospy.Subscriber('/robot1/move_base/local_costmap/costmap', OccupancyGrid, self.my_position)

		# find my orientation
		sub = rospy.Subscriber('/robot1/odom', Odometry, self.get_rotation)

		# take the global map data from topic
		map_info = rospy.Subscriber('/robot1/move_base/global_costmap/costmap', OccupancyGrid, self.map_occupancy)

		rospy.sleep(1)
		# check 3 pixels ahead 
		print self.x
		print self.y
		# print self.map_data[(floor(self.x)+200)*(floor(self.y)+200)]

		if self.x != 0:
			# print len(self.map_data)
			# print self.map_data[(int) ((math.floor(self.x)+200)*400 + (math.floor(self.y)+200))]
			print self.degrees
			# # -45 <= degrees < 0
			# if self.degrees < 0 and self.degrees >= -45.0:

			# # -90 <= degrees < -45
			# elif self.degrees < -45.0 and self.degrees >= -90.0:

			# # -135 <= degrees < -90
			# elif self.degrees < -90.0 and self.degrees >= -135.0:

			# # -180 <= degrees < -135
			# elif self. degrees < -135.0 and self.degrees >= -180.0:
	
			# # 0 <= degrees < 45
			# elif self.degrees < 45.0 and self.degrees >= 0:
			# 	print self.map_data[(int) ((math.floor(self.x) - 5 +200)*400 + (math.floor(self.y)+200))]
			# 	if self.map_data[(int) ((math.floor(self.x) - 5 +200)*400 + (math.floor(self.y)+200))] != 0:
			# 		return False
			
			# # 45 <= degrees < 90
			# elif self.degrees >= 45.0 and self.degrees < 90.0:
			# 	print self.map_data[(int) ((math.floor(self.x)+200)*400 + (math.floor(self.y) + 5 +200))]
			# 	if self.map_data[(int) ((math.floor(self.x)+200)*400 + (math.floor(self.y) + 5 +200))] != 0:
			# 		return False

			# # 90 <= degrees < 135
			# elif self.degrees >= 90.0 and self.degrees < 135.0: 
			
			# # 135 <= degrees < 180
			# elif self.degrees >= 135 and self.degrees < 180:
				

		return True

	def map_occupancy(self, m_data):
		self.map_data = m_data.data


	def my_position(self, data): # find my current position
		self.x = data.info.origin.position.x
		self.y = data.info.origin.position.y
		# rospy.loginfo('Position %f, %f', x,y)


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


