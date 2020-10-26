#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from multiple_turtlebots_nav.msg import Num
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ReadMetr():

	def __init__(self):
		print 'Reading Metrics'
		rospy.on_shutdown(self.shutdown)
		self.roll = self.pitch = self.yaw = 0.0


		# rospy.init_node('PublicMetr', anonymous=False)

		rospy.init_node('readmetr', anonymous=True)

		rate = rospy.Rate(10)
		num = Num()

		self.my_odom = Odometry

		sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

		self.sub_metrics = rospy.Subscriber('/metrics_topic', Num, self.callback)

		
		while not rospy.is_shutdown(): # Ctr + c
			# rospy.loginfo('while')

			rate.sleep()
		

			# self.sub_metrics.subscribe(num, )


	def shutdown(self):
		rospy.loginfo("Stop Reading")
		rospy.sleep(1)

	def callback(self, data):

		my_position_x = int(self.my_odom.pose.pose.position.x)
		my_position_y = int(self.my_odom.pose.pose.position.y)

		rospy.loginfo('x = %d', my_position_x)
		rospy.loginfo('y = %d', my_position_y)

		rospy.loginfo("printing")
		print data.data[(my_position_y+150) * 400 + my_position_x+200]


	def get_rotation(self, msg): # the orientation of the turtlebot
		self.my_odom = msg
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
		# rospy.loginfo(self.yaw)



if __name__ == '__main__':
	try:
		ReadMetr()
	except:
		rospy.loginfo("Reading node terminated.")


