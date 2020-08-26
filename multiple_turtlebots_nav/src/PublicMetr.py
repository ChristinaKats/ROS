#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from multiple_turtlebots_nav.msg import Num
import itertools
import csv


class PublicMetr():

	def __init__(self):
		print 'Publishing Metrics'
		rospy.on_shutdown(self.shutdown)

		self.sub_metrics = rospy.Publisher('metrics_topic', Num, queue_size=10)
		rospy.init_node('metrics', anonymous=True)

		rate = rospy.Rate(1)

		with open('/home/christina/catkin_ws/src/multiple_turtlebots_nav/src/data_source_one.csv', mode='r') as csv_file:
			csv_reader = csv.reader(csv_file, delimiter=',')
			data_tuple = [tuple(row) for row in csv_reader]

			columns = len(data_tuple[0])
			rows = len(data_tuple)

		num = Num()
		num.width = columns
		num.height = rows

		a = []

		for data in data_tuple:
			for data1 in data:
				a.append(int(float(data1)))
		
		num.data = a
		rospy.loginfo(num.data)
		
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


