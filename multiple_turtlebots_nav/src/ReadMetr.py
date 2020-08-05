#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from multiple_turtlebots_nav.msg import Num

class ReadMetr():

	def __init__(self):
		print 'Reading Metrics'
		rospy.on_shutdown(self.shutdown)

		# rospy.init_node('PublicMetr', anonymous=False)

		rospy.init_node('readmetr', anonymous=True)

		rate = rospy.Rate(1)
		num = Num()
		
		while not rospy.is_shutdown(): # Ctr + c
			self.sub_metrics = rospy.Subscriber('metrics_topic', Num, self.callback)
			rate.sleep()
		

			# self.sub_metrics.subscribe(num, )


	def shutdown(self):
		rospy.loginfo("Stop Reading")
		rospy.sleep(1)

	def callback(self, data):
		rospy.loginfo("printing")
		print data.width



if __name__ == '__main__':
	try:
		ReadMetr()
	except:
		rospy.loginfo("Reading node terminated.")


