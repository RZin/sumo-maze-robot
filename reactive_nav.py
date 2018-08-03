#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

obstacle_distance_left = None
obstacle_distance_front = None
obstacle_distance_right = None
sense_combination = [0,0,0]

DIST_THR = 1.2

robot_stopped = False

rand_choice = (-1,1) # right or left



def laserCallback_0(msgIn):

    global obstacle_distance_left
    # left
    ranges_left = np.array(msgIn.ranges)
    obstacle_distance_left = ranges_left.min() # msgIn.range_min

    # rospy.loginfo("obstacle_distance_left: {}".format(obstacle_distance_left))


def laserCallback_1(msgIn):

    global obstacle_distance_front
    # mid
    ranges_front = np.array(msgIn.ranges)
    obstacle_distance_front = ranges_front.min() # msgIn.range_min

    # rospy.loginfo("obstacle_distance_front: {}".format(obstacle_distance_front))

def laserCallback_2(msgIn):

    global obstacle_distance_right
    # right
    ranges_right = np.array(msgIn.ranges)
    obstacle_distance_right = ranges_right.min() # msgIn.range_min

    # rospy.loginfo("obstacle_distance_right: {}".format(obstacle_distance_right))



def main():
    global obstacle_distance_left, obstacle_distance_front,obstacle_distance_right, robot_stopped, sense_combination

    rospy.init_node('reactive_nav')

    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,
                          queue_size=100)



    cmd_vel_msg = Twist()
    robot_stopped = True

    r = rospy.Rate(4)
    while not rospy.is_shutdown():

        laser_sub0 = rospy.Subscriber('base_scan_0', LaserScan,
                                     laserCallback_0, queue_size=100)
        laser_sub1 = rospy.Subscriber('base_scan_1', LaserScan,
                                     laserCallback_1, queue_size=100)
        laser_sub2 = rospy.Subscriber('base_scan_2', LaserScan,
                                     laserCallback_2, queue_size=100)


        sense_combination = [int(obstacle_distance_left<DIST_THR),  int(obstacle_distance_front<DIST_THR) , int(obstacle_distance_right<DIST_THR)]

        rospy.loginfo("sense_comb={}".format(sense_combination))

        # # =====================random follower
        # if sense_combination == [1, 1, 0] or sense_combination == [1, 1, 1] or sense_combination == [0, 1, 0]:
        #     # turn right
        #     rospy.loginfo("[0,1,0]  [1,1,0] or [1,1,1]")
        #     cmd_vel_msg.linear.x = 0.0
        #     cmd_vel_msg.angular.z = random.choice(rand_choice)
        #
        #     if robot_stopped:
        #         rospy.loginfo("Stopping")
        #         robot_stopped = True
        #
        # elif sense_combination == [0, 0, 1] or sense_combination == [0, 1, 1]:
        #     # turn left or right and move forward
        #     rospy.loginfo("[0,0,0] [0,0,1] or [0,1,1]")
        #     cmd_vel_msg.linear.x = 1.0
        #     cmd_vel_msg.angular.z = -0.5
        #
        #     if robot_stopped:
        #         rospy.loginfo("Moving")
        #         robot_stopped = False
        #
        # elif sense_combination == [0, 0, 0] or sense_combination == [1, 1, 0] or sense_combination == [1, 0, 1]:
        #     rospy.loginfo("[1,1,0] or [1,0,1]")
        #     cmd_vel_msg.linear.x = 2.0
        #     cmd_vel_msg.angular.z = 0.0
        #
        # else:  # move forward
        #     '''[1,1,0] [1,0,1]'''
        #     rospy.loginfo("else")

        # =====================left follower

        if sense_combination == [1, 1, 0] or sense_combination == [1, 1, 1] or sense_combination == [0, 1, 0]:
            # turn right
            rospy.loginfo("[0,1,0]  [1,1,0] or [1,1,1]")
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = -0.5

            if robot_stopped:
                rospy.loginfo("Stopping")
                robot_stopped = True

        elif sense_combination == [0, 0, 0] or sense_combination == [0, 0, 1] or sense_combination == [0, 1, 1]:
            # turn left and move forward
            rospy.loginfo("[0,0,0] [0,0,1] or [0,1,1]")
            cmd_vel_msg.linear.x = 1.0
            cmd_vel_msg.angular.z = 1.0

            if robot_stopped:
                rospy.loginfo("Moving")
                robot_stopped = False

        elif sense_combination == [1, 1, 0] or sense_combination == [1, 0, 1]:
            # move forward
            rospy.loginfo("[1,1,0] or [1,0,1]")
            cmd_vel_msg.linear.x = 2.0
            cmd_vel_msg.angular.z = 0.0

        else:  # move forward
            '''[1,1,0] [1,0,1]'''
            rospy.loginfo("else")

        cmd_vel_pub.publish(cmd_vel_msg)
        # rospy.spin()

        r.sleep()




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass