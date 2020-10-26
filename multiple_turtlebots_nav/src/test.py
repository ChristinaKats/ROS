#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


import math
import numpy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from multiple_turtlebots_nav.msg import Num
from multiple_turtlebots_nav.msg import Best_Metric, Best_Metric_send
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import csv


class GoToPose():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.init_node("RobotController")
        rospy.Subscriber("/robot1/odom", Odometry, self.newOdom)
        self.pub = rospy.Publisher("/robot1/cmd_vel_mux/input/navi", Twist, queue_size = 10)
        self.speed = Twist()
        self.r = rospy.Rate(5)

        self.go()


    def newOdom(self, msg):
    
        # rospy.loginfo('mpike')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(q)
        self.theta = euler[2]


    def go(self):
        goal = Point()
        goal.x = 3.3
        goal.y = 3.3

        rospy.sleep(1)
        inc_x = goal.x - self.x
        inc_y = goal.y - self.y
        angle_to_goal = math.atan2(inc_y, inc_x)
        rospy.loginfo(abs(angle_to_goal - self.theta))
        while abs(angle_to_goal - self.theta) > 0.1 :
            scale=1.5
            # dir=(angle_to_goal-self.theta) /abs( angle_to_goal - self.theta)
            # angSpeed= min(0.5, abs( angle_to_goal-self.theta) / scale)
            self.speed.angular.z = 0.2
            self.speed.linear.x = 0
            self.pub.publish(self.speed)
            self.r.sleep()

            # inc_x = goal.x - self.x
            # inc_y = goal.y - self.y
            # angle_to_goal = math.atan2(inc_y, inc_x)

            # rospy.loginfo('while')
        rospy.loginfo('finished')

        distance=math.sqrt(pow(goal.x - self.x,2) + pow(goal.y - self.y, 2))

        # rospy.loginfo('mexri edo kala')
        self.speed.angular.z = 0
        while distance >= 0.7:
            distance=math.sqrt(pow(goal.x-self.x,2) + pow(goal.y-self.y, 2))
            # rospy.loginfo(distance)
            self.speed.linear.x = 0.5
            self.speed.angular.z = 0
            self.pub.publish(self.speed)
            self.r.sleep()
            self.speed.linear.x = 0.0
            self.speed.angular.z =0.0
            self.pub.publish(self.speed)

# rospy.loginfo('telos')

if __name__ == '__main__':
    GoToPose()
