#!/usr/bin/env python

import rospy
import sys
from nav_msgs.msg import OccupancyGrid, Odometry
from multiple_turtlebots_nav.msg import Num
from multiple_turtlebots_nav.msg import Best_Metric_send
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class PublishLocal():
    def __init__(self):

        rospy.init_node(sys.argv[1], anonymous=False)


        self.odom_topic = sys.argv[1] + '/odom'
        self.local_topic = sys.argv[1] + '/local_best'
        self.local_best = 0.0
        self.my_position_x = 0
        self.my_position_y = 0
        self.metrics = Num()
        rospy.Subscriber (self.odom_topic, Odometry, self.get_rotation)
        rospy.Subscriber('/metrics_topic', Num, self.read_metrics) # read robot's metric

        self.publish()


    def publish(self): # publish values

        global_best = rospy.Publisher(self.local_topic, Best_Metric_send, queue_size=10) # write to /global_best
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            num = Best_Metric_send()
            num.metr = self.local_best
            rospy.loginfo(self.local_best)
            num.position_x = self.my_position_x
            num.position_y = self.my_position_y

            global_best.publish(num)
            r.sleep()

    def get_rotation(self, msg): # the orientation of the turtlebot
        self.my_position_x = int(msg.pose.pose.position.x)
        self.my_position_y = int(msg.pose.pose.position.y)

        self.my_odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
    
    def read_metrics(self, data):
        self.metrics = data
        metr = self.metrics.data[(self.my_position_x+200) * 400 + self.my_position_y+200]

        if metr > self.local_best:
            self.local_best = metr


if __name__ == '__main__':
    PublishLocal()
