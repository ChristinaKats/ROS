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
from multiple_turtlebots_nav.msg import Best_Metric
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import csv

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


class Meandre():

    def __init__(self):
        self.start = time.time()
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('Meandre2', anonymous=False)

        self.distances = [] # laserscan distances
        self.my_odom = Odometry() # my position (odom topic)
        # self.my_odom.pose.pose.position.x = 1
        # self.my_odom.pose.pose.position.y = 1
        # [self.my_odom.pose.pose.orientation.x, self.my_odom.pose.pose.orientation.y, self.my_odom.pose.pose.orientation.z, self.my_odom.pose.pose.orientation.w] = quaternion_from_euler(0,0, 1.8415926) 

        self.move_base = actionlib.SimpleActionClient("robot2/move_base", MoveBaseAction)

        self.roll = self.pitch = self.yaw = 0.0
        self.metrics = Num
        self.global_best_value = Best_Metric
        self.global_best_value.metric = 0.0

        self.local_best = 0.0


        self.friends_pos = Odometry

        self.scans = rospy.Subscriber('/robot2/scan', LaserScan, self.my_distances)

        self.cmd_vel = rospy.Publisher('/robot2/cmd_vel_mux/input/navi', Twist, queue_size=10)

        self.set_odom = rospy.Publisher('/robot2/odom', Odometry, queue_size=10)
        self.set_odom.publish(self.my_odom)

        rospy.Subscriber ('/robot2/odom', Odometry, self.get_rotation)
        rospy.Subscriber ('/robot1/odom', Odometry, self.get_friends_pos)

        self.global_best = rospy.Publisher('/robot2/local_best', Best_Metric, queue_size=10) # write to /global_best
        # self.global_best.publish(self.global_best_value)

        # self.local_best = rospy.Publisher('/robot1/local_best', Best_Metric, queue_size=10) # write to /local_best
        rospy.Subscriber('/robot1/local_best', Best_Metric, self.read_global_best) # read from /global_best
        # rospy.Subscriber('/global_best', Best_Metric, self.read_global_best)

        self.sub_metrics = rospy.Subscriber('/metrics_topic', Num, self.read_metrics) # read robot's metric

        self.csv_rows = []

        self.prev_pos_x = 0
        self.prev_pos_y = 0

        self.metric_in_position = 0

        rospy.sleep(1)

        # self.go_forward()
        find_wall = self.find_random_wall()
        rospy.loginfo(find_wall)
        if find_wall == 0:
            # rospy.loginfo('brika toixo')
            hand_on_wall = self.right_hand_on_wall() 
            if hand_on_wall == 0: # turn to touch the wall with the right hand
                # rospy.loginfo('following wall')
                if self.meandre_walkthrough() == 1:
                    self.go_to_metrics()
                elif self.meandre_walkthrough() == 2:
                    self.follow_turtle()

            elif hand_on_wall == 2: # if the other turtlebot found a metric
                self.follow_turtle()

            else: # you found a metric
                # rospy.loginfo('found metric')
                self.go_to_metrics()
        elif find_wall == 1:
            self.go_to_metrics()

        else :
            self.follow_turtle()

    def follow_turtle(self):
        rospy.loginfo('follow other turtle')

        # find orientation
        my_position_x = int(self.my_odom.pose.pose.position.x)
        my_position_y = int(self.my_odom.pose.pose.position.y)

        goal_position_x = self.global_best_value.position_x
        goal_position_y = self.global_best_value.Position_y


        # Customize the following values so they are appropriate for your location
        position = {'x': goal_position_x , 'y' : goal_position_y}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        self.goto(position, quaternion)


    def goto(self, pos, quat):

        # Send a goal
        # self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
        self.move_base.send_goal(goal)

		# Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

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

            curr_metric = Best_Metric
            # metrics in this position
            curr_metric.metric = self.metrics.data[(my_position_y+150) * 400 + my_position_x+210]
            curr_metric.position_x = my_position_x
            curr_metric.position_y = my_position_y

            if self.global_best_value.metric < curr_metric.metric: # if you found a better global best value, tell the other turtlebot
                self.global_best_value.metric = curr_metric.metric
                self.global_best_value.position_x = my_position_x
                self.global_best_value.position_y = my_position_y

                self.global_best.publish(self.global_best_value)


            if self.local_best < curr_metric.metric: # if you found a better global best value, tell the other turtlebot
                self.local_best = curr_metric.metric


            #get orientation of turtlebot
            if math.degrees(self.yaw) > -45 and math.degrees(self.yaw) < 45: # looking left 
                rospy.loginfo('left')
                front_x = my_position_x + 1
                front_y = my_position_y

                right_x = my_position_x 
                right_y = my_position_y - 1

                left_x = my_position_x
                left_y = my_position_y + 1

            elif math.degrees(self.yaw) <= -45 and math.degrees(self.yaw) >-135: # looking up
                rospy.loginfo('up')

                front_x = my_position_x
                front_y = my_position_y - 1

                right_x = my_position_x - 1
                right_y = my_position_y

                left_x = my_position_x + 1
                left_y = my_position_y

            elif  (math.degrees(self.yaw) < 0 and math.degrees(self.yaw) <= -135) or (math.degrees(self.yaw) > 0 and math.degrees(self.yaw) > 135): # looking right
                rospy.loginfo('right')

                front_x = my_position_x - 1
                front_y = my_position_y

                right_x = my_position_x 
                right_y = my_position_y + 1

                left_x = my_position_x
                left_y = my_position_y - 1


            elif math.degrees(self.yaw) > 45 and math.degrees(self.yaw) < 135: #looking down
                rospy.loginfo('down')

                front_x = my_position_x
                front_y = my_position_y + 1

                right_x = my_position_x + 1
                right_y = my_position_y

                left_x = my_position_x - 1
                left_y = my_position_y

            # rospy.loginfo('my %d, %d, %f',my_position_x, my_position_y, self.metrics.data[(my_position_y+150) * 400 + my_position_x+210])

            # rospy.loginfo('front %d, %d, %f',front_x, front_y, self.metrics.data[(front_y+150) * 400 + front_x+210])
            # rospy.loginfo('left %d, %d, %f',left_x, left_y, self.metrics.data[(left_y+150) * 400 + left_x+210])
            # rospy.loginfo('right %d, %d, %f',right_x, right_y, self.metrics.data[(right_y+150) * 400 + right_x+210])

            # rospy.loginfo(front_x)

            # rospy.loginfo('before ifs')

            # rospy.loginfo(self.metrics.data[(front_y+150) * 400 + front_x+210])
            # rospy.loginfo(self.metrics.data[(left_y+150) * 400 + left_x+210])
            
            if self.metrics.data[(front_y+150) * 400 + front_x+210] >= self.metrics.data[(left_y+150) * 400 + left_x+210] and self.metrics.data[(front_y+150) * 400 + front_x+210] >= curr_metric.metric:
                # rospy.loginfo('111111')
                # if self.metrics.data[(front_y+150) * 400 + front_x+210] >= self.metrics.data[(right_y+150) * 400 + right_x+210] :
                    # front has the best metric
                    #continue straight

                  
                # else :
                if self.metrics.data[(front_y+150) * 400 + front_x+210] < self.metrics.data[(right_y+150) * 400 + right_x+210] :
                    # rospy.loginfo('turn right')
                    self.turn_90_degrees_right()

            elif self.metrics.data[(left_y+150) * 400 + left_x+210] > self.metrics.data[(front_y+150) * 400 + front_x+210] and self.metrics.data[(left_y+150) * 400 + left_x+210] >= curr_metric.metric:
                # rospy.loginfo('222222')
                
                if self.metrics.data[(left_y+150) * 400 + left_x+210] >= self.metrics.data[(right_y+150) * 400 + right_x+210] :
                    # rospy.loginfo('turn left')

                    self.turn_90_degrees_left()
                else: 
                    # rospy.loginfo('turn right')

                    self.turn_90_degrees_right()
            elif self.metrics.data[(right_y+150) * 400 + right_x+210] > curr_metric.metric:
                # rospy.loginfo('turn right')
                self.turn_90_degrees_right()

            else : 
                rospy.loginfo('finish go metrics')
                break

            move_cmd = Twist()
            move_cmd.linear.x = 0.2 # go forward at 0.5 m/s
            move_cmd.angular.z = 0 # go straight only
            self.cmd_vel.publish(move_cmd)
            # rospy.loginfo('continue straight')
            r.sleep()


    def get_friends_pos(self, data):
        self.friends_pos = data

    def distance_from_other_turtle(self):
        x = (self.my_odom.pose.pose.position.x - self.friends_pos.pose.pose.position.x) * (self.my_odom.pose.pose.position.x - self.friends_pos.pose.pose.position.x)
        y = (self.my_odom.pose.pose.position.y - self.friends_pos.pose.pose.position.y) * (self.my_odom.pose.pose.position.y - self.friends_pos.pose.pose.position.y)

        distance = math.sqrt(x + y)
        # rospy.loginfo('dist = %f', distance)
        return distance


    def find_random_wall(self):
        while self.check_next_pos_scan(): # free space in front of the turtlebot
            # rospy.loginfo('sth')
            # prev_metric = self.metric_in_position

            move_cmd = Twist()
            move_cmd.linear.x = 0.5 # go forward at 0.5 m/s
            move_cmd.angular.z = 0 # go straight only
            self.cmd_vel.publish(move_cmd)

            if self.metric_in_position != 0:
                return 1

            # rospy.loginfo(self.local_best)
            # rospy.loginfo(self.global_best_value.metric)

            if self.local_best != self.global_best_value.metric : # other turtlebot found a metric
                return 2

        return 0


    def check_next_pos_scan(self):
        # rospy.loginfo(self.distances[320])
		# If there is an odstacle 1.5m ahead, you should stop moving
        if math.isnan(min(self.distances)) == False and min(self.distances) < 1.5 and min(self.distances) > 0: 
	        return False
        else: # else, free space continue straight
            return True


    def right_hand_on_wall(self): # turn to touch the wall with the right hand
        while len(self.distances) == 0: # no scans registered yet, wait
            # rospy.loginfo("sleeping")
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

            if self.local_best != self.global_best_value.metric :
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
            # rospy.loginfo('dist = %f', distance_from_init)
            while self.distances[0] > 1.7 and self.distances[0] < 4 and (math.isnan(self.distances[320]) or self.distances[320] > 1.5): #follow wall
                move_cmd.linear.x = 0.5 # go straight 
                move_cmd.angular.z = 0 # do not turn 
                self.cmd_vel.publish(move_cmd)

                curr_position_x = self.my_odom.pose.pose.position.x
                curr_position_y = self.my_odom.pose.pose.position.y

                distance_from_init = math.sqrt((curr_position_x - init_position_x)*(curr_position_x - init_position_x) + (curr_position_y - init_position_y)*(curr_position_y - init_position_y))
                if distance_from_init > 4:
                    flag = True
            
                # rospy.loginfo('distance = %f', distance_from_init)

                if distance_from_init <= 4 and flag == True:
                    break

                if self.metric_in_position != 0:
                    return 1

                if self.local_best != self.global_best_value.metric:
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
            
            # rospy.loginfo('distance = %f', distance_from_init)

            if self.metric_in_position != 0:
                return 1


            if self.local_best != self.global_best_value.metric:
                return 2

        return 0

    def read_metrics(self, data):
        self.metrics = data

        # rospy.loginfo(data)

        # current position
        my_position_x = int(self.my_odom.pose.pose.position.x)
        my_position_y = int(self.my_odom.pose.pose.position.y)

        # rospy.loginfo('x = %d', my_position_x)
        # rospy.loginfo('y = %d', my_position_y)

        # rospy.loginfo('array pos = %d', (my_position_x+100) * 400 + my_position_y+100)

        curr_metric = Best_Metric
        # metrics in this position
        curr_metric.metric = self.metrics.data[(my_position_y+150) * 400 + my_position_x+210]
        curr_metric.position_x = my_position_x
        curr_metric.position_y = my_position_y

        self.metric_in_position = curr_metric.metric 


        # rospy.loginfo('metric %f', curr_metric.metric)

        # if self.global_best_value.metric < curr_metric.metric:
        #     self.global_best.publish(curr_metric)
            # rospy.loginfo('new best')

        curr_time = time.time() - self.start
        
        if my_position_x != self.prev_pos_x or my_position_y != self.prev_pos_y:
            row = []
            row.append(curr_time)
            row.append(my_position_x)
            row.append(my_position_y)
            row.append(curr_metric.metric)

            # rospy.loginfo(row)

            # rospy.loginfo(row)

            self.csv_rows.append(row)

            self.prev_pos_x = my_position_x
            self.prev_pos_y = my_position_y

    def read_global_best(self, best):
        self.global_best_value = best
        rospy.loginfo('best = %f', self.global_best_value.metric)

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
        # for i in range(3):
        #     move_cmd.linear.x = -0.5 # do not move 
        #     move_cmd.angular.z = 0 # turn only
        #     self.cmd_vel.publish(move_cmd)

        while True: 
            if self.distances.index(min(self.distances)) > 370 or self.distances.index(min(self.distances)) < 300:
                rospy.loginfo('face the wall')
                self.face_the_wall()

            self.turn_90_degrees_left()
            result = self.go_forward_5m(meters, 'r')
            if result == False:
                rospy.loginfo('result = False')

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
                rospy.loginfo('face the wall')
                self.face_the_wall()


            self.turn_90_degrees_right()
            result = self.go_forward_5m(meters, 'l')
            if result == False:
                rospy.loginfo('result = False')

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

            # rospy.loginfo(pos_min_element)
            r.sleep()
            

        move_cmd.linear.x = 0 # do not move 
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)


    def go_forward_5m(self, meters, hand):
        rospy.loginfo('go forward 5 meters')

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
                # rospy.loginfo('r')
                # if you got to close to the wall, turn left
                if math.isnan(self.distances[0]) == False and self.distances[0] <= 3.5 and self.distances[0] > 0:
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = 0.1 # turn left 
                    self.cmd_vel.publish(move_cmd)
                    init_yaw = self.yaw

                    # rospy.loginfo('nai')

                elif self.distances[0] > 6: # if you break away from the wall, turn right
                    move_cmd.linear.x = 0 # stop moving 
                    move_cmd.angular.z = -0.1 # turn right 
                    self.cmd_vel.publish(move_cmd)
                    init_yaw = self.yaw


            else:
                # rospy.loginfo('l')

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
            # rospy.loginfo(math.degrees(self.yaw))
            current_yaw = self.yaw

            if current_yaw - init_yaw > 0.1:
                # rospy.loginfo('turn right')
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = -0.1 # do not turn
                self.cmd_vel.publish(move_cmd)

            elif current_yaw - init_yaw < -0.1:
                # rospy.loginfo('turn left')

                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = 0.1 # do not turn
                self.cmd_vel.publish(move_cmd)


            if abs(current_yaw - init_yaw) > 1:
                init_yaw = current_yaw

            curr_position_x = self.my_odom.pose.pose.position.x
            curr_position_y = self.my_odom.pose.pose.position.y

            distance_from_init = math.sqrt((curr_position_x - init_position_x)*(curr_position_x - init_position_x) + (curr_position_y - init_position_y)*(curr_position_y - init_position_y))
            # if distance_from_init > 2:
                # flag = True 
            
            r.sleep()
            
        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning
        self.cmd_vel.publish(move_cmd)
        # rospy.loginfo('go forward end')

        return True


    def go_forward(self):
        rospy.loginfo('go forward')
        rospy.sleep(1)

        close_to_wall = False

        init_yaw = self.yaw

        move_cmd = Twist()
        r = rospy.Rate(10)
        while math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0:

            # rospy.loginfo(min(self.distances))
            if math.isnan(min(self.distances)) == False and min(self.distances) < 3 and min(self.distances) > 0:
                close_to_wall = True
                # rospy.loginfo('plisiase poly')
                break
                # edo to break prepei na allaksei 
            
            move_cmd.linear.x = 0.3 # go strauight 
            move_cmd.angular.z = 0 # do not turn
            self.cmd_vel.publish(move_cmd)


            if self.metric_in_position != 0: # a metric found
                return 1

            if self.local_best != self.global_best_value.metric:
                return 2

            current_yaw = self.yaw
            if current_yaw - init_yaw > 0.1:
                # rospy.loginfo('turn right')
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = -0.1 # do not turn
                self.cmd_vel.publish(move_cmd)

            elif current_yaw - init_yaw < -0.1:
                # rospy.loginfo('turn left')
                move_cmd.linear.x = 0 # go strauight 
                move_cmd.angular.z = 0.1 # do not turn
                self.cmd_vel.publish(move_cmd)


            if abs(current_yaw - init_yaw) > 1:
                init_yaw = current_yaw
            
            r.sleep()

        if close_to_wall == True:

            if self.distance_from_other_turtle() < 3:
                for i in range(5):
                    rospy.sleep(1)

                return self.go_forward()


            # rospy.loginfo('too close, %d', self.distances.index(min(self.distances)))
            if self.distances.index(min(self.distances)) < 320:
                # rospy.loginfo('too close 1')

                while math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0: 

                    while self.distances[0] > 3 and self.distances[0] < 6 and (math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0): #follow wall
                        move_cmd.linear.x = 0.3 # go straight 
                        move_cmd.angular.z = 0 # do not turn 
                        self.cmd_vel.publish(move_cmd)
                        # rospy.loginfo('while straight')

                        if self.metric_in_position != 0:
                            return 1

                        if self.local_best != self.global_best_value.metric:
                            return 2

                    # if you got to close to the wall, turn left
                    if (math.isnan(self.distances[0]) == False and self.distances[0] <= 3 and self.distances[0] > 0) or (math.isnan(self.distances[320]) == False and self.distances[320] <= 3 and self.distances[320] > 0):
                        move_cmd.linear.x = 0 # stop moving 
                        move_cmd.angular.z = 0.1 # turn left 
                        self.cmd_vel.publish(move_cmd)
                        # rospy.loginfo('if 1')


                    elif self.distances[0] > 6: # if you break away from the wall, turn right
                        move_cmd.linear.x = 0 # stop moving 
                        move_cmd.angular.z = -0.1 # turn right 
                        self.cmd_vel.publish(move_cmd)
                        # rospy.loginfo('if else')


            else: 
                # rospy.loginfo('too close 2')

                while  math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0: 

                    while self.distances[639] > 1.7 and self.distances[639] < 4 and (math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0): #follow wall
                        move_cmd.linear.x = 0.3 # go straight 
                        move_cmd.angular.z = 0 # do not turn 
                        self.cmd_vel.publish(move_cmd)

                        if self.metric_in_position != 0:
                            return 1

                        if self.local_best != self.global_best_value.metric:
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
        # rospy.loginfo('go forward end')

        return 0


    def get_rotation(self, msg): # the orientation of the turtlebot
        self.my_odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        # rospy.loginfo(self.yaw)

    
    def turn_90_degrees_left(self):
        rospy.loginfo('turn left')

        move_cmd = Twist()
        # get the current orientation
        current_orient = math.degrees(self.yaw)
        end_orient = math.degrees(self.yaw)

        # rospy.sleep(1)
        r = rospy.Rate(10)

        rospy.loginfo(current_orient)
        if current_orient >= 0 and current_orient <= 90:
            while end_orient - current_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient > 90 and current_orient <= 180:
            count = 0

            while count < 5 or end_orient > 0 or (math.fabs(end_orient) + current_orient > 270): # turn 90 degreess
                # rospy.loginfo('mpainei')
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                count += 1
                # rospy.login
                r.sleep()

        elif current_orient < 0 and current_orient > -90:
            while end_orient + math.fabs(current_orient) < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        else :
            while end_orient - current_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = 0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning   
        self.cmd_vel.publish(move_cmd)


    def turn_90_degrees_right(self):
        rospy.loginfo('turn right')

        

        move_cmd = Twist()
        # get the current orientation
        current_orient = math.degrees(self.yaw)
        end_orient = math.degrees(self.yaw)

        # rospy.sleep(1)

        r = rospy.Rate(10)

        rospy.loginfo('curr = %f',current_orient)

        if current_orient > 0 and current_orient < 90:
            rospy.loginfo('0 - 90')
            # rospy.loginfo('curr = %f',current_orient)

            # rospy.loginfo(math.fabs(end_orient) + current_orient)
            while current_orient - end_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                # rospy.loginfo(math.fabs(end_orient) + current_orient)
                r.sleep()
            # rospy.loginfo('end = %f',current_orient)


        elif current_orient > 90 and current_orient <= 180:
            rospy.loginfo('90 - 180')

            while current_orient - end_orient < 87: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        elif current_orient < 0 and current_orient > -90:
            rospy.loginfo('-90 - 0')
            while current_orient - end_orient < 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()

        else :
            rospy.loginfo('else')
            while end_orient < 0 or end_orient - (current_orient + 180) > 90: # turn 90 degreess
                move_cmd.linear.x = 0 # do not move 
                move_cmd.angular.z = -0.1 # turn only
                self.cmd_vel.publish(move_cmd)
                end_orient = math.degrees(self.yaw)
                r.sleep()


        rospy.loginfo('end = %f', end_orient)

        move_cmd.linear.x = 0 # do not move     
        move_cmd.angular.z = 0 # stop turning   
        self.cmd_vel.publish(move_cmd)


    def shutdown(self): # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        move_cmd = Twist()
        move_cmd.linear.x = 0 # stop moving
        move_cmd.angular.z = 0 # stop turning
        # self.cmd_vel.publish(move_cmd)

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

