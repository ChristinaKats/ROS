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
import csv

class Meandre():

    def __init__(self):
        self.start = time.time()
        rospy.on_shutdown(self.shutdown)
        rospy.init_node('Meandre', anonymous=False)

        self.distances = [] # laserscan distances
        self.my_odom = Odometry # my position (odom topic)
        self.roll = self.pitch = self.yaw = 0.0
        self.metrics = Num
        self.global_best_value = Best_Metric
        self.global_best_value.metric = -1

        self.scans = rospy.Subscriber('/scan', LaserScan, self.my_distances)

        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)

        self.global_best = rospy.Publisher('/global_best', Best_Metric, queue_size=10)
        current_best = rospy.Subscriber('/global_best', Best_Metric, self.read_global_best)

        self.sub_metrics = rospy.Subscriber('/metrics_topic', Num, self.read_metrics)

        self.csv_rows = []

        self.prev_pos_x = 0
        self.prev_pos_y = 0

        rospy.sleep(1)

        # self.go_forward()
        self.find_random_wall()
        self.right_hand_on_wall() # turn to touch the wall with the right hand
        self.meandre_walkthrough()

    def find_random_wall(self):
        while self.check_next_pos_scan(): # free space in front of the turtlebot
            # rospy.loginfo('sth')
            move_cmd = Twist()
            move_cmd.linear.x = 0.5 # go forward at 0.5 m/s
            move_cmd.angular.z = 0 # go straight only
            self.cmd_vel.publish(move_cmd)

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
        self.turn_until_right_hand_on_wall()

        # follow the wall
        self.walk_by_the_wall()

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
        self.turn_90_degrees_left()

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


        rospy.loginfo('metric %f', curr_metric.metric)

        if self.global_best_value.metric < curr_metric.metric:
            self.global_best.publish(curr_metric)
            rospy.loginfo('new best')

        curr_time = time.time() - self.start
        
        if my_position_x != self.prev_pos_x or my_position_y != self.prev_pos_y:
            row = []
            row.append(curr_time)
            row.append(my_position_x)
            row.append(my_position_y)
            row.append(curr_metric.metric)

            rospy.loginfo(row)


            # rospy.loginfo(row)

            self.csv_rows.append(row)

            self.prev_pos_x = my_position_x
            self.prev_pos_y = my_position_y

        # rospy.loginfo(self.csv_rows)
        # rospy.loginfo('sth----------------------')

        
    def read_global_best(self, best):
        self.global_best_value = best

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

        meters = 3

        threshhold = 2
        count = 0
        self.face_the_wall()

        move_cmd = Twist()
        for i in range(3):
            move_cmd.linear.x = -0.5 # do not move 
            move_cmd.angular.z = 0 # turn only
            self.cmd_vel.publish(move_cmd)

        while True: 
            if self.distances.index(min(self.distances)) > 370 or self.distances.index(min(self.distances)) < 300:
                rospy.loginfo('face the wall')
                self.face_the_wall()

            self.turn_90_degrees_left()
            result = self.go_forward_3m(meters, 'r')
            if result == False:
                rospy.loginfo('result = False')

                self.turn_90_degrees_left()
                self.go_forward()

                self.turn_90_degrees_left()
                self.go_forward_3m(meters, 'r')

                count += 1

            
            
            self.turn_90_degrees_left()
            self.go_forward()


            if self.distances.index(min(self.distances)) > 370 or self.distances.index(min(self.distances)) < 300:
                rospy.loginfo('face the wall')
                self.face_the_wall()


            self.turn_90_degrees_right()
            result = self.go_forward_3m(meters, 'l')
            if result == False:
                rospy.loginfo('result = False')

                self.turn_90_degrees_right()
                self.go_forward()

                self.turn_90_degrees_right()
                self.go_forward_3m(meters, 'l')

                count += 1

            self.turn_90_degrees_right()
            self.go_forward()

            if count == threshhold:
                return


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


    def go_forward_3m(self, meters, hand):
        rospy.loginfo('go forward 3 meters')

        rospy.sleep(1)
        init_yaw = self.yaw
        rospy.loginfo(init_yaw)

        move_cmd = Twist()

        distance_from_init = 0
        
        init_position_x = self.my_odom.pose.pose.position.x
        init_position_y = self.my_odom.pose.pose.position.y

        r = rospy.Rate(10)

        while distance_from_init < 3: 
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

            rospy.loginfo('too close, %d', self.distances.index(min(self.distances)))
            if self.distances.index(min(self.distances)) < 320:
                # rospy.loginfo('too close 1')

                while math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0: 

                    while self.distances[0] > 3 and self.distances[0] < 6 and (math.isnan(self.distances[320]) or self.distances[320] > 3 or self.distances[320] < 0): #follow wall
                        move_cmd.linear.x = 0.3 # go straight 
                        move_cmd.angular.z = 0 # do not turn 
                        self.cmd_vel.publish(move_cmd)
                        # rospy.loginfo('while straight')

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
        filename = "/home/christina/catkin_ws/src/multiple_turtlebots_nav/src/statistics.csv"

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

