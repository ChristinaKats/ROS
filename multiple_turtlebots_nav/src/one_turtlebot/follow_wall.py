#!/usr/bin/env python

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan



class WallFollow(): # follow the wall until you return at your initial position
    # We assume that the turtlebot has already reached a wall and the goal is to follow it

    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('WallFollow', anonymous=False)

        self.distances = [] # laserscan distances
        self.my_odom = Odometry # my position (odom topic)
        self.roll = self.pitch = self.yaw = 0.0
        self.scans = rospy.Subscriber('/scan', LaserScan, self.my_distances)
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

        r = rospy.Rate(1)
        self.right_hand_on_wall() # turn to touch the wall with the right hand

    def get_rotation(self, msg): # the orientation of the turtlebot
        self.my_odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def right_hand_on_wall(self): # turn to touch the wall with the right hand
        while len(self.distances) == 0: # no scans registered yet, wait
            rospy.loginfo("sleeping")
            rospy.sleep(1)

        # Turn so that your right hand faces the wall
        self.turn_until_right_hand_on_wall()

        # follow the wall
        self.walk_by_the_wall()

    def my_distances(self, laser_data):
        self.distances = laser_data.ranges
        # rospy.loginfo(self.distances)
        max_dist = self.find_max()
        # rospy.loginfo('max = %d', max_dist)

        for i in range(len(self.distances)):
            if math.isnan(self.distances[i]):
                temp_list = list(self.distances)
                temp_list[i] = 10*max_dist
                self.distances = tuple(temp_list)
        
        return

    def turn_until_right_hand_on_wall(self):
        min_element = min(self.distances) # closest obstacle
        pos_min_element = self.distances.index(min_element) # relative position of obstacle and turtlebot 
        
        move_cmd = Twist()

        # face the wall
        while pos_min_element < 310 or pos_min_element > 330: # turn to face the obstacle
            # turn the turtlebot right  
            move_cmd.linear.x = 0 # do not move 
            move_cmd.angular.z = 0.1 # turn only
            self.cmd_vel.publish(move_cmd)

            min_element = min(self.distances)
            pos_min_element = self.distances.index(min_element)

        move_cmd.linear.x = 0 # do not move 
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)

        # turn to touch the wall with the right hand
        self.turn_90_degrees()

    def turn_90_degrees(self):
        move_cmd = Twist()
        # get the current orientation
        current_orient = math.degrees(self.yaw)
        end_orient = math.degrees(self.yaw)
        while end_orient - current_orient <= 89: # turn 90 degreess
            move_cmd.linear.x = 0 # do not move 
            move_cmd.angular.z = 0.1 # turn only
            self.cmd_vel.publish(move_cmd)

            end_orient = math.degrees(self.yaw)

        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)

    def walk_by_the_wall(self):
        # if you touch the wall, go straight
        move_cmd = Twist()
        distance_from_init = 0

        # initial position of the turtlebot when it first touched the wall
        init_position_x = self.my_odom.pose.pose.position.x
        init_position_y = self.my_odom.pose.pose.position.y

        flag = False # boolean variable indicates if the turtlebot got at least 2m away from the initial position
        # it is used for the following condition. At first flag == False, which means that turtlebot is at the beggining of its trip. 
        # when the turtlebot moves away of its initial position, flag = True. While flag == True, if distance < 2, 
        # it shows that the turtlebot returned close to the initial position, so it is time to stop following the wall  

        while distance_from_init > 2 or flag == False: # until you return to the initial position, follow the wall  

            while self.distances[0] > 1.7 and self.distances[0] < 4 and (math.isnan(self.distances[320]) or self.distances[320] > 1.5): #follow wall
                move_cmd.linear.x = 0.5 # go straight 
                move_cmd.angular.z = 0 # do not turn 
                self.cmd_vel.publish(move_cmd)

                curr_position_x = self.my_odom.pose.pose.position.x
                curr_position_y = self.my_odom.pose.pose.position.y

                distance_from_init = math.sqrt((curr_position_x - init_position_x)*(curr_position_x - init_position_x) + (curr_position_y - init_position_y)*(curr_position_y - init_position_y))
                if distance_from_init > 2:
                    flag = True
            
                # rospy.loginfo('distance = %f', distance_from_init)

                if distance_from_init <= 2 and flag == True:
                    break

            # if you got to close to the wall, turn left
            if (math.isnan(self.distances[0]) == False and self.distances[0] <= 1.7 and self.distances[0] > 0) or (math.isnan(self.distances[320]) == False and self.distances[320] <= 1.5 and self.distances[320] > 0):
                move_cmd.linear.x = 0 # stop moving 
                move_cmd.angular.z = 0.1 # turn left 
                self.cmd_vel.publish(move_cmd)

            elif self.distances[0] > 4: # if you break away from the wall, turn right
                move_cmd.linear.x = 0 # stop moving 
                move_cmd.angular.z = -0.1 # turn right 
                self.cmd_vel.publish(move_cmd)

            elif math.isnan(self.distances[0]) or self.distances[0] < 0: # if wall ended, go staight for 2m and turn until you reach the a wall
                init_pose_x = self.my_odom.pose.pose.position.x
                init_pose_y = self.my_odom.pose.pose.position.y
                distance = 0
                while distance < 2: # go straight for 2m
                    move_cmd.linear.x = 0.5 # move straight 
                    move_cmd.angular.z = 0.0 # do not turn  
                    self.cmd_vel.publish(move_cmd)

                    curr_pose_x = self.my_odom.pose.pose.position.x
                    curr_pose_y = self.my_odom.pose.pose.position.y

                    distance = math.sqrt((curr_pose_x - init_pose_x)*(curr_pose_x - init_pose_x) + (curr_pose_y - init_pose_y)*(curr_pose_y - init_pose_y))
                
                while math.isnan(self.distances[0]) or self.distances[0] < 0: # until you find a wall at your right
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = -0.1 # turn right 
                    self.cmd_vel.publish(move_cmd)

            # current position of turtlebot
            curr_position_x = self.my_odom.pose.pose.position.x
            curr_position_y = self.my_odom.pose.pose.position.y

            # distance from the initial position
            distance_from_init = math.sqrt((curr_position_x - init_position_x)*(curr_position_x - init_position_x) + (curr_position_y - init_position_y)*(curr_position_y - init_position_y))
            if distance_from_init > 2: # if the turtlebot moved at least 2m from the initial position
                flag = True
            
            rospy.loginfo('distance = %f', distance_from_init)


    def shutdown(self): # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        move_cmd = Twist()
        move_cmd.linear.x = 0 # stop moving
        move_cmd.angular.z = 0 # stop turning

        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def find_max(self): # max element of distances list
        max_dist = -1
        for i in range(len(self.distances)):
            if math.isnan(self.distances[i]) == False:
                if max_dist < self.distances[i]: 
                    max_dist = self.distances[i]

        return max_dist



if __name__ == '__main__':
    try:
        WallFollow()
    except:
        rospy.loginfo("WallFollow node terminated.")




