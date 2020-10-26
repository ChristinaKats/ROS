#!/usr/bin/env python

import rospy
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
from time import sleep
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


class Meandre():

    def __init__(self):
        self.start = time.time()
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('Meandre1', anonymous=False)

        self.distances = [] # laserscan distances
        self.my_odom = Odometry() # my position (odom topic)
        
        self.move_base = actionlib.SimpleActionClient("robot1/move_base", MoveBaseAction)
        
        self.roll = self.pitch = self.yaw = 0.0
        self.metrics = Num
        self.other_turtles_best_value = Best_Metric
        self.other_turtles_best_value.metr = 0.0
        self.other_turtles_best_value.position_x = 0
        self.other_turtles_best_value.position_y = 0

        self.theta = 0.0
        self.local_best = 0.0
        self.friends_pos = Odometry

        self.scans = rospy.Subscriber('/robot1/scan', LaserScan, self.my_distances)
        self.cmd_vel = rospy.Publisher('/robot1/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.set_odom = rospy.Publisher('/robot1/odom', Odometry, queue_size=10)
        self.set_odom.publish(self.my_odom)

        rospy.Subscriber ('/robot1/odom', Odometry, self.get_rotation)
        rospy.Subscriber ('/robot2/odom', Odometry, self.get_friends_pos)

        rospy.Subscriber('/robot2/local_best', Best_Metric_send, self.read_global_best) # read from /global_best
        self.csv_rows = []

        self.prev_pos_x = 0
        self.prev_pos_y = 0
        self.metric_in_position = 0
        self.sub_metrics = rospy.Subscriber('/metrics_topic', Num, self.read_metrics) # read robot's metric

        rospy.sleep(1)

        find_wall = self.find_random_wall()
        if find_wall == 0:
            hand_on_wall = self.right_hand_on_wall() 
            if hand_on_wall == 0: # turn to touch the wall with the right hand
                if self.meandre_walkthrough() == 1: # metric found
                    self.go_to_metrics()
                elif self.meandre_walkthrough() == 2: # other turtle found metric
                    self.follow_turtle()

            elif hand_on_wall == 2: # if the other turtlebot found a metric
                self.follow_turtle()

            else: # you founhd a metric
                # rospy.loginfo('found metric')
                self.go_to_metrics()
        elif find_wall == 1:
            self.go_to_metrics()
        else:
            self.follow_turtle()
                
    def follow_turtle(self):
        goal_position_x = self.other_turtles_best_value.position_x
        goal_position_y = self.other_turtles_best_value.position_y

        position = Point()
        position.x = goal_position_x
        position.y = goal_position_y

        if self.goto(position):
            self.go_to_metrics()
        
    def recalculate(self):
        goal_position_x = self.other_turtles_best_value.position_x
        goal_position_y = self.other_turtles_best_value.position_y
        position = Point()
        position.x = goal_position_x
        position.y = goal_position_y
        return position


    def goto(self, goal):
        temp_metr = self.other_turtles_best_value.metr
        inc_x = goal.x - self.my_odom.pose.pose.position.x
        inc_y = goal.y - self.my_odom.pose.pose.position.y
        angle_to_goal = math.atan2(inc_y, inc_x)
        rospy.loginfo(abs(angle_to_goal - self.theta))
        speed = Twist()
        r = rospy.Rate(5)

        while abs(angle_to_goal - self.theta) > 0.1 :
            speed.angular.z = 0.2
            speed.linear.x = 0
            self.cmd_vel.publish(speed)
            r.sleep()

        distance=math.sqrt(pow(goal.x - self.my_odom.pose.pose.position.x,2) + pow(goal.y - self.my_odom.pose.pose.position.y, 2))

        speed.angular.z = 0
        while self.metric_in_position == 0:
            while(self.distance_from_other_turtle() < 2.0):
                sleep(1)

            while abs(angle_to_goal - self.theta) > 0.1 :
                speed.angular.z = 0.2
                speed.linear.x = 0
                self.cmd_vel.publish(speed)
                r.sleep()

            distance=math.sqrt(pow(goal.x-self.my_odom.pose.pose.position.x,2) + pow(goal.y-self.my_odom.pose.pose.position.y, 2))
            speed.linear.x = 0.3
            speed.angular.z = 0
            self.cmd_vel.publish(speed)
            r.sleep()

            if self.metric_in_position != 0:
                return True
            
            if self.other_turtles_best_value.metr - temp_metr  > 0.01:
                goal = self.recalculate()
                inc_x = goal.x - self.my_odom.pose.pose.position.x
                inc_y = goal.y - self.my_odom.pose.pose.position.y
                angle_to_goal = math.atan2(inc_y, inc_x)
                temp_metr = self.other_turtles_best_value.metr

        speed.linear.x = 0.0
        speed.angular.z =0.0
        self.cmd_vel.publish(speed)

        if self.metric_in_position != 0:
            return True
                
        return False


    def go_to_metrics(self): # if robot finds a metric > 0 then it movew towards the greater metric found so far
        # find max metric arround turtlebot
        # current position
        front_x = 0
        front_y = 0

        right_x = 0
        right_y = 0

        left_x = 0
        left_y = 0

        r = rospy.Rate(1)

        while(1):
            my_position_x = int(self.my_odom.pose.pose.position.x)
            my_position_y = int(self.my_odom.pose.pose.position.y)

            curr_metric = Best_Metric_send()
            # metrics in this position
            curr_metric.metr = self.metrics.data[(my_position_x+200) * 400 + my_position_y+200]
            curr_metric.position_x = my_position_x
            curr_metric.position_y = my_position_y
            if self.local_best < curr_metric.metr: # if you found a better global best value, tell the other turtlebot
                self.local_best = curr_metric.metr


            #get orientation of turtlebot
            if math.degrees(self.yaw) > -45 and math.degrees(self.yaw) < 45: # looking left 
                front_x = my_position_x + 1
                front_y = my_position_y

                right_x = my_position_x 
                right_y = my_position_y - 1

                left_x = my_position_x
                left_y = my_position_y + 1

            elif math.degrees(self.yaw) <= -45 and math.degrees(self.yaw) >-135: # looking up

                front_x = my_position_x
                front_y = my_position_y - 1

                right_x = my_position_x - 1
                right_y = my_position_y

                left_x = my_position_x + 1
                left_y = my_position_y

            elif  (math.degrees(self.yaw) < 0 and math.degrees(self.yaw) <= -135) or (math.degrees(self.yaw) > 0 and math.degrees(self.yaw) > 135): # looking right
                front_x = my_position_x - 1
                front_y = my_position_y

                right_x = my_position_x 
                right_y = my_position_y + 1

                left_x = my_position_x
                left_y = my_position_y - 1


            elif math.degrees(self.yaw) > 45 and math.degrees(self.yaw) < 135: #looking down
                front_x = my_position_x
                front_y = my_position_y + 1

                right_x = my_position_x + 1
                right_y = my_position_y

                left_x = my_position_x - 1
                left_y = my_position_y

            if self.check_distances(320-20, 320+20) and self.check_distances(0, 100) and self.check_distances(500, 639): # all positions available
                if self.metrics.data[(front_x+200) * 400 + front_y+200] >= self.metrics.data[(left_x+200) * 400 + left_y+200] and self.metrics.data[(front_x+200) * 400 + front_y+200] >= curr_metric.metr:
                    if self.friends_pos.pose.pose.position.x != right_x or self.friends_pos.pose.pose.position.y != right_y: # turtlebot can go front
                        if self.metrics.data[(front_x+200) * 400 + front_y+200] < self.metrics.data[(right_x+200) * 400 + right_y+200]:
                            self.turn_90_degrees_right()

                elif self.metrics.data[(left_x+200) * 400 + left_y+200] > self.metrics.data[(front_x+200) * 400 + front_y+200] and self.metrics.data[(left_x+200) * 400 + left_y+200] >= curr_metric.metr:
                    if self.metrics.data[(left_x+200) * 400 + left_y+200] >= self.metrics.data[(right_x+200) * 400 + right_y+200]:
                        self.turn_90_degrees_left()
                    else: 
                        self.turn_90_degrees_right()
                elif self.metrics.data[(right_x+200) * 400 + right_y+200] > curr_metric.metr:
                    self.turn_90_degrees_right()

                else: 
                    if self.other_turtles_best_value.metr >= self.curr_metric.metr:
                        self.follow_turtle()
                        break

            elif self.check_distances(0, 100) and self.check_distances(500, 639): #right and left available
                if self.metrics.data[(left_x+200) * 400 + left_y+200] >= self.metrics.data[(right_x+200) * 400 + right_y+200]:
                        self.turn_90_degrees_left()
                else: 
                    self.turn_90_degrees_right()

            elif self.check_distances(0, 100) and self.check_distances(300, 340): #left and front available
                if self.metrics.data[(front_x+200) * 400 + front_y+200] < self.metrics.data[(right_x+200) * 400 + right_y+200]:
                    self.turn_90_degrees_right()
            
            elif self.check_distances(500, 639) and self.check_distances(300, 340): #right and front available
                if self.metrics.data[(front_x+200) * 400 + front_y+200] < self.metrics.data[(right_x+200) * 400 + right_y+200]:
                    self.turn_90_degrees_right()

            elif self.check_distances(500, 639): #right available
                self.turn_90_degrees_right()

            elif self.check_distances(0, 100): #left available
                self.turn_90_degrees_left()

            elif self.check_distances(300, 340) != True: #front unavailable
                if self.other_turtles_best_value.metr >= self.curr_metric.metr:
                    self.follow_turtle()
                    break
            move_cmd = Twist()
            move_cmd.linear.x = 0.2 # go forward at 0.5 m/s
            move_cmd.angular.z = 0 # go straight only
            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def get_friends_pos(self, data):
        self.friends_pos = data

    def distance_from_other_turtle(self):
        x = (self.my_odom.pose.pose.position.x - self.friends_pos.pose.pose.position.x) * (self.my_odom.pose.pose.position.x - self.friends_pos.pose.pose.position.x)
        y = (self.my_odom.pose.pose.position.y - self.friends_pos.pose.pose.position.y) * (self.my_odom.pose.pose.position.y - self.friends_pos.pose.pose.position.y)

        distance = math.sqrt(x + y)
        return distance

    def find_random_wall(self):
        while self.check_next_pos_scan(): # free space in front of the turtlebot
            move_cmd = Twist()
            move_cmd.linear.x = 0.5 # go forward at 0.5 m/s
            move_cmd.angular.z = 0 # go straight only
            self.cmd_vel.publish(move_cmd)

            if self.metric_in_position != 0:
                return 1

            if self.other_turtles_best_value.metr != 0 : # other turtlebot found a metric
                return 2

        return 0

    def check_distances(self, start_range, end_range):
        for i in range(start_range, end_range):
            if math.isnan(self.distances[i]) != True and self.distances[i] > 0 and self.distances[i] < 2:
                return False
        return True

    def check_next_pos_scan(self):
		# If there is an odstacle 1.5m ahead, you should stop moving
        if math.isnan(min(self.distances)) == False and min(self.distances) < 1.5 and min(self.distances) > 0: 
	        return False
        else: # else, free space continue straight
            return True


    def right_hand_on_wall(self): # turn to touch the wall with the right hand
        while len(self.distances) == 0: # no scans registered yet, wait
            rospy.sleep(1)

        # Turn so that your right hand faces the wall
        if self.turn_until_right_hand_on_wall(): # if other turtlebot found metric
            return 2

        # follow the wall
        return self.walk_by_the_wall()

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

            if self.other_turtles_best_value.metr != 0 :
                return True

        move_cmd.linear.x = 0 # do not move 
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)

        # turn to touch the wall with the right hand
        self.turn_90_degrees_left()

        return False

    def find_max(self): # max element of distances list
        max_dist = -1
        for i in range(len(self.distances)):
            if math.isnan(self.distances[i]) == False:
                if max_dist < self.distances[i]: 
                    max_dist = self.distances[i]

        return max_dist

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

        while distance_from_init > 4 or flag == False: # until you return to the initial position, follow the wall  
            while self.distances[0] > 1.7 and self.distances[0] < 4 and (math.isnan(self.distances[320]) or self.distances[320] > 1.5): #follow wall
                move_cmd.linear.x = 0.5 # go straight 
                move_cmd.angular.z = 0 # do not turn 
                self.cmd_vel.publish(move_cmd)

                curr_position_x = self.my_odom.pose.pose.position.x
                curr_position_y = self.my_odom.pose.pose.position.y

                distance_from_init = math.sqrt((curr_position_x - init_position_x)*(curr_position_x - init_position_x) + (curr_position_y - init_position_y)*(curr_position_y - init_position_y))
                if distance_from_init > 4:
                    flag = True
            
                if distance_from_init <= 4 and flag == True:
                    break

                if self.metric_in_position != 0:
                    return 1

                if self.other_turtles_best_value.metr != 0:
                    return 2

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
            if distance_from_init > 4: # if the turtlebot moved at least 2m from the initial position
                flag = True
            
            if self.metric_in_position != 0:
                return False

            if self.other_turtles_best_value.metr != 0:
                return 2

        return 0

    def read_metrics(self, data):
        self.metrics = data

        # current position
        my_position_x = int(self.my_odom.pose.pose.position.x)
        my_position_y = int(self.my_odom.pose.pose.position.y)

        curr_metric = Best_Metric_send()
        # metrics in this position
        curr_metric.metr = self.metrics.data[(my_position_x+200) * 400 + my_position_y+200]
        curr_metric.position_x = my_position_x
        curr_metric.position_y = my_position_y

        self.metric_in_position = curr_metric.metr

        curr_time = time.time() - self.start
        
        if my_position_x != self.prev_pos_x or my_position_y != self.prev_pos_y:
            row = []
            row.append(curr_time)
            row.append(my_position_x)
            row.append(my_position_y)
            row.append(curr_metric.metr)

            self.csv_rows.append(row)

            self.prev_pos_x = my_position_x
            self.prev_pos_y = my_position_y

    def read_global_best(self, best):
        # self.other_turtles_best_value = Best_Metric_send()
        self.other_turtles_best_value = best

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
        meters = 5
        threshhold = 2
        count = 0
        self.face_the_wall()

        move_cmd = Twist()
        while True: 
            if self.distances.index(min(self.distances)) > 370 or self.distances.index(min(self.distances)) < 300:
                self.face_the_wall()

            self.turn_90_degrees_left()
            result = self.go_forward_5m(meters, 'r')
            if result == False:
                self.turn_90_degrees_left()
                forward = self.go_forward() 
                if forward == 1:
                    return 1

                if forward == 2:
                    return 2

                self.turn_90_degrees_left()
                self.go_forward_5m(meters, 'r')

                count += 1

            self.turn_90_degrees_left()

            forward = self.go_forward() 
            if forward == 1:
                return 1

            if forward == 2:
                return 2
        

            if self.distances.index(min(self.distances)) > 370 or self.distances.index(min(self.distances)) < 300:
                self.face_the_wall()


            self.turn_90_degrees_right()
            result = self.go_forward_5m(meters, 'l')
            if result == False:
                self.turn_90_degrees_right()
                forward = self.go_forward() 
                if forward == 1:
                    return 1

                if forward == 2:
                    return 2

                self.turn_90_degrees_right()
                self.go_forward_5m(meters, 'l')

                count += 1

            self.turn_90_degrees_right()
            forward = self.go_forward() 
            if forward == 1:
                return 1

            if forward == 2:
                return 2

            if count == threshhold:
                return 0


    def face_the_wall(self):
        min_element = min(self.distances) # closest obstacle
        pos_min_element = self.distances.index(min_element) # relative position of obstacle and turtlebot 
        
        move_cmd = Twist()

        r = rospy.Rate(15)
        # face the wall
        while pos_min_element < 310 or pos_min_element > 330: # turn to face the obstacle
            # turn the turtlebot right  
            move_cmd.linear.x = 0 # do not move 
            move_cmd.angular.z = 0.1 # turn only
            self.cmd_vel.publish(move_cmd)

            min_element = min(self.distances)
            pos_min_element = self.distances.index(min_element)

            r.sleep()

        move_cmd.linear.x = 0 # do not move 
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)


    def go_forward_5m(self, meters, hand):
        rospy.sleep(1)
        init_yaw = self.yaw
        rospy.loginfo(init_yaw)
        move_cmd = Twist()
        distance_from_init = 0
        
        init_position_x = self.my_odom.pose.pose.position.x
        init_position_y = self.my_odom.pose.pose.position.y

        r = rospy.Rate(10)

        while distance_from_init < 5: 
            if math.isnan(self.distances[320]) == False and self.distances[320] <= 3 and self.distances[320] > 0:
                return False

            if hand == 'r':
                # if you got to close to the wall, turn left
                if math.isnan(self.distances[0]) == False and self.distances[0] <= 3.5 and self.distances[0] > 0:
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = 0.1 # turn left 
                    self.cmd_vel.publish(move_cmd)
                    init_yaw = self.yaw

                elif self.distances[0] > 6: # if you break away from the wall, turn right
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = -0.1 # turn right 
                    self.cmd_vel.publish(move_cmd)
                    init_yaw = self.yaw

            else:
                # if you got to close to the wall, turn right
                if math.isnan(self.distances[639]) == False and self.distances[639] <= 3.5 and self.distances[639] > 0:
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = -0.1 # turn left 
                    self.cmd_vel.publish(move_cmd)
                    init_yaw = self.yaw

                elif self.distances[639] > 6: # if you break away from the wall, turn right
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = 0.1 # turn left 
                    self.cmd_vel.publish(move_cmd)
                    init_yaw = self.yaw

            move_cmd.linear.x = 0.3 # go strauight 
            move_cmd.angular.z = 0 # do not turn
            self.cmd_vel.publish(move_cmd)
            current_yaw = self.yaw

            if current_yaw - init_yaw > 0.1:
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = -0.1 # do not turn
                self.cmd_vel.publish(move_cmd)

            elif current_yaw - init_yaw < -0.1:
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = 0.1 # do not turn
                self.cmd_vel.publish(move_cmd)

            if abs(current_yaw - init_yaw) > 1:
                init_yaw = current_yaw

            curr_position_x = self.my_odom.pose.pose.position.x
            curr_position_y = self.my_odom.pose.pose.position.y
            distance_from_init = math.sqrt((curr_position_x - init_position_x)*(curr_position_x - init_position_x) + (curr_position_y - init_position_y)*(curr_position_y - init_position_y))
            r.sleep()
            
        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)
        return True

    def go_forward(self):
        rospy.sleep(1)
        close_to_wall = False
        init_yaw = self.yaw
        move_cmd = Twist()
        r = rospy.Rate(10)
        while math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0:
            if math.isnan(min(self.distances)) == False and min(self.distances) < 3 and min(self.distances) > 0:
                close_to_wall = True
                break
            
            move_cmd.linear.x = 0.3 # go strauight 
            move_cmd.angular.z = 0 # do not turn
            self.cmd_vel.publish(move_cmd)
            if self.metric_in_position != 0: # a metric found
                return 1

            if self.other_turtles_best_value.metr != 0:
                return 2

            current_yaw = self.yaw
            if current_yaw - init_yaw > 0.1:
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = -0.1 # do not turn
                self.cmd_vel.publish(move_cmd)

            elif current_yaw - init_yaw < -0.1:
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = 0.1 # do not turn
                self.cmd_vel.publish(move_cmd)

            if abs(current_yaw - init_yaw) > 1:
                init_yaw = current_yaw
            r.sleep()

        if close_to_wall == True:
            if self.distance_from_other_turtle() < 3:
                self.turn_90_degrees_right()
                if self.go_forward_5m(5, 'r') == False:
                    self.turn_90_degrees_left()
                    self.turn_90_degrees_left()
                    self.go_forward_5m(5, 'l')
                    self.turn_90_degrees_right()
                self.turn_90_degrees_left()
                return self.go_forward()

            if self.distances.index(min(self.distances)) < 320:
                while math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0: 
                    while self.distances[0] > 3 and self.distances[0] < 6 and (math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0): #follow wall
                        move_cmd.linear.x = 0.3 # go straight 
                        move_cmd.angular.z = 0 # do not turn 
                        self.cmd_vel.publish(move_cmd)

                        if self.metric_in_position != 0:
                            return 1

                        if self.local_best != self.global_best_value.metr:
                            return 2

                    # if you got to close to the wall, turn left
                    if (math.isnan(self.distances[0]) == False and self.distances[0] <= 3 and self.distances[0] > 0) or (math.isnan(self.distances[320]) == False and self.distances[320] <= 3 and self.distances[320] > 0):
                        move_cmd.linear.x = 0 # stop moving 
                        move_cmd.angular.z = 0.1 # turn left 
                        self.cmd_vel.publish(move_cmd)

                    elif self.distances[0] > 6: # if you break away from the wall, turn right
                        move_cmd.linear.x = 0 # stop moving 
                        move_cmd.angular.z = -0.1 # turn right 
                        self.cmd_vel.publish(move_cmd)
            else: 
                while  math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0: 

                    while self.distances[639] > 1.7 and self.distances[639] < 4 and (math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0): #follow wall
                        move_cmd.linear.x = 0.3 # go straight 
                        move_cmd.angular.z = 0 # do not turn 
                        self.cmd_vel.publish(move_cmd)

                        if self.metric_in_position != 0:
                            return 1

                        if self.other_turtles_best_value.metr != 0:
                            return 2

                    # if you got to close to the wall, turn left
                    if (math.isnan(self.distances[639]) == False and self.distances[639] <= 2 and self.distances[639] > 0) or (math.isnan(self.distances[320]) == False and self.distances[320] <= 3 and self.distances[320] > 0):
                        move_cmd.linear.x = 0 # stop moving 
                        move_cmd.angular.z = -0.1 # turn left 
                        self.cmd_vel.publish(move_cmd)

                    elif self.distances[639] > 4: # if you break away from the wall, turn right
                        move_cmd.linear.x = 0 # stop moving 
                        move_cmd.angular.z = 0.1 # turn right 
                        self.cmd_vel.publish(move_cmd)

        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)
        return 0

    def get_rotation(self, msg): # the orientation of the turtlebot
        self.my_odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(q)
        self.theta = euler[2]
    
    def turn_90_degrees_left(self):
        move_cmd = Twist()
        # get the current orientation
        current_orient = math.degrees(self.yaw)
        end_orient = math.degrees(self.yaw)
        r = rospy.Rate(10)

        rospy.loginfo(current_orient)
        if current_orient >= 0 and current_orient <= 90:
            while end_orient - current_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.2 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient > 90 and current_orient <= 180:
            count = 0

            while count < 5 or end_orient > 0 or (math.fabs(end_orient) + current_orient > 270): # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.2 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                count += 1
                r.sleep()

        elif current_orient < 0 and current_orient > -90:
            while end_orient + math.fabs(current_orient) < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.2 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        else :
            while end_orient - current_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.2 # turn only
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

        r = rospy.Rate(10)
        if current_orient > 0 and current_orient < 90:
            while current_orient - end_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.2 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient > 90 and current_orient <= 180:
            while current_orient - end_orient < 87: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.2 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient < 0 and current_orient > -90:
            while current_orient - end_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.2 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        else :
            while end_orient < 0 or end_orient - (current_orient + 180) > 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.2 # turn only
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
        # field names 
        fields = ['Time', 'Position_x', 'Position_y', 'Metrics'] 

        # name of csv file 
        filename = "/home/christina/catkin_ws/src/multiple_turtlebots_nav/src/statistics_1.csv"

        # writing to csv file 
        with open(filename, 'w') as csvfile: 
            # creating a csv writer object 
            csvwriter = csv.writer(csvfile) 
            csvwriter.writerow(fields) 
            for i in self.csv_rows:
                csvwriter.writerow(i) 

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Meandre()
    except:
        rospy.loginfo("Meandre node terminated.")

