#!/usr/bin/env python

import rospy
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid


class FindPoint():
	def __init__(self):
		# self.sub = rospy.Subscriber("map_metadata", MapMetaData, callback)
		self.sub = rospy.Subscriber("/robot1/move_base/local_costmap/costmap", OccupancyGrid, printGrid)



def printGrid(msg):
	print 'In Grid'
	data = msg.data
	x = msg.info.width
	y = msg.info.height

	# for i in range(0, x):
		# for j in range(0, y):
			# print (data[i*x + j]),

	print data


	# for pos in data:
	# 	if pos != -1:
	# 		print(pos), 
	# print msg.data

def callback(msg):
	w = msg.width
	h = msg.height
	print 'In callback func', w,  h

	print msg.origin.position.x, msg.origin.position.y 


if __name__ == '__main__':
	rospy.init_node('listener', anonymous=True)
	FindPoint()
	rospy.spin()