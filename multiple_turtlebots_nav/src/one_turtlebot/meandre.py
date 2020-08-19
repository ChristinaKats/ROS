#!/usr/bin/env python

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan


class Meandre():

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('Meandre', anonymous=False)

        self.distances = [] # laserscan distances
        self.my_odom = Odometry # my position (odom topic)
        self.roll = self.pitch = self.yaw = 0.0

        self.scans = rospy.Subscriber('/scan', LaserScan, self.my_distances)

        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

        rospy.sleep(1)

        self.meandre_walkthrough()


    def my_distances(self, laser_data):
        self.distances = laser_data.ranges
        max_dist = self.find_max()

        for i in range(len(self.distances)):
            if math.isnan(self.distances[i]):
                temp_list = list(self.distances)
                temp_list[i] = 10*max_dist
                self.distances = tuple(temp_list)
        
        return

    def find_max(self): # max element of distances list
        max_dist = -1
        for i in range(len(self.distances)):
            if math.isnan(self.distances[i]) == False:
                if max_dist < self.distances[i]: 
                    max_dist = self.distances[i]

        return max_dist

    def meandre_walkthrough(self):
        self.face_the_wall()
        rospy.loginfo('face the wall')
        self.turn_90_degrees_left()
        rospy.loginfo('turn left out')

        while True: 
            self.turn_90_degrees_left()
            self.go_forward()
            self.turn_90_degrees_left()
            self.go_forward()
            self.turn_90_degrees_right()
            self.go_forward()


            # break


            # self.turn_90_degrees_right()
        # self.go_down()


    def face_the_wall(self):
        min_element = min(self.distances) # closest obstacle
        pos_min_element = self.distances.index(min_element) # relative position of obstacle and turtlebot 
        
        move_cmd = Twist()

        r = rospy.Rate(15)
        # face the wall
        while pos_min_element < 310 or pos_min_element > 330: # turn to face the obstacle
            # turn the turtlebot right  
            move_cmd.linear.x = 0 # do not move 
            move_cmd.angular.z = 0.2 # turn only
            self.cmd_vel.publish(move_cmd)

            min_element = min(self.distances)
            pos_min_element = self.distances.index(min_element)

            # rospy.loginfo(pos_min_element)
            r.sleep()
            

        move_cmd.linear.x = 0 # do not move 
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)



    def go_forward(self):
        rospy.loginfo('go forward')

        move_cmd = Twist()
        r = rospy.Rate(10)
        while math.isnan(self.distances[320]) or self.distances[320] > 1.5 or self.distances[320] < 0:
            move_cmd.linear.x = 0.3 # go strauight 
            move_cmd.angular.z = 0 # do not turn
            self.cmd_vel.publish(move_cmd)

            rospy.loginfo(min(self.distances))
            if math.isnan(min(self.distances)) == False and min(self.distances) < 1.5 and min(self.distances) > 0:
                break
                # edo to break prepei na allaksei 
            
            r.sleep()
            
        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)
        rospy.loginfo('go forward end')


    def get_rotation(self, msg): # the orientation of the turtlebot
        self.my_odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)


    def turn_90_degrees_left(self):
        # rospy.sleep(1)
        move_cmd = Twist()
        # get the current orientation
        current_orient = math.degrees(self.yaw)
        end_orient = math.degrees(self.yaw)

        # rospy.sleep(1)
        r = rospy.Rate(10)
        if current_orient >= 0 and current_orient <= 90:
            while end_orient - current_orient <= 89: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient > 90 and current_orient <= 180:
            count = 0
            while count < 5 or math.fabs(end_orient) - math.fabs(current_orient) >= 0: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                count += 1
                r.sleep()

        elif current_orient < 0 and current_orient > -90:
            while end_orient - math.fabs(current_orient) <= 0: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        else :
            while end_orient - current_orient < 89: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning   
        self.cmd_vel.publish(move_cmd)


    def turn_90_degrees_right(self):
        move_cmd = Twist()
        # get the current orientation
        current_orient = math.degrees(self.yaw)
        end_orient = math.degrees(self.yaw)

        # rospy.sleep(1)

        r = rospy.Rate(10)
        if current_orient >= 0 and current_orient <= 90:
            while math.fabs(end_orient) - current_orient <= 0: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient > 90 and current_orient <= 180:
            while current_orient - end_orient < 89: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient < 0 and current_orient > -90:
            while current_orient - end_orient < 89: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        else :
            while end_orient + current_orient <= 0: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning   
        self.cmd_vel.publish(move_cmd)


    def shutdown(self): # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        move_cmd = Twist()
        move_cmd.linear.x = 0 # stop moving
        move_cmd.angular.z = 0 # stop turning
        # self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Meandre()
    except:
        rospy.loginfo("Meandre node terminated.")

