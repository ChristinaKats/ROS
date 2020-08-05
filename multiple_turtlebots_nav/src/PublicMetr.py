#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from multiple_turtlebots_nav.msg import Num

class PublicMetr():

	def __init__(self):
		print 'Publishing Metrics'
		rospy.on_shutdown(self.shutdown)

		# rospy.init_node('PublicMetr', anonymous=False)

		self.sub_metrics = rospy.Publisher('metrics_topic', Num, queue_size=10)
		rospy.init_node('metrics', anonymous=True)

		rate = rospy.Rate(1)

		# num = "hello"

		num = Num()
		num.width = 2
		num.height = 2

		# num = Num()
		# num.width = 4
		# num.height = 10
		# num.data = numpy.zeros(shape=(400,400))

		num.data = [1, 2, 3, 4, 5, 6]
		while not rospy.is_shutdown(): # Ctr + c
			rospy.loginfo("publishing")
			self.sub_metrics.publish(num)
			rate.sleep()


	def shutdown(self):
		rospy.loginfo("Stop Publishing")

		rospy.sleep(1)


if __name__ == '__main__':
	try:
		PublicMetr()
	except:
		rospy.loginfo("Publishing node terminated.")


