#!/usr/bin/python
# -*- coding: utf-8 -*-

# todo mb add service of commands (follow_left, follow_right, random)

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random

navigation_parameters = {

    'fast_lin_speed': 8.0, # m/s
    'mid_lin_speed': 2.5,
    'slow_lin_speed': 0.8,
    
    'fast_ang_speed': np.pi/0.5, # rad/s
    'mid_ang_speed': np.pi/2,
    'slow_ang_speed': np.pi/3,
    
    'close_distance_threshold': 1.0, # m
    'far_distance_threshold': 1.5,
    'cycle_threshold': 125,

    'pub_topic': '/cmd_vel',
    'sub_topic0': '/base_scan_0',
    'sub_topic1': '/base_scan_1',
    'sub_topic2': '/base_scan_2'
}


class Navigator:

    def __init__(self, algorithm = 'follow_left', fast_lin_speed=1.0, mid_lin_speed=0.5,slow_lin_speed=0.2, fast_ang_speed=np.pi/2,mid_ang_speed=np.pi/4, slow_ang_speed=np.pi/8, close_distance_threshold = 0.5, far_distance_threshold = 1.5, cycle_threshold = 5, pub_topic = '/cmd_vel', sub_topic0 = '/base_scan_0', sub_topic1='/base_scan_1', sub_topic2='/base_scan_2'):

        # check correctness of params
        included_algorithms = ['follow_left', 'follow_right', 'random']
        assert algorithm.lower() in included_algorithms, "unknown algorithm"
        assert fast_lin_speed >= mid_lin_speed >= slow_lin_speed
        assert fast_ang_speed >= mid_ang_speed >= slow_ang_speed
        assert far_distance_threshold >= close_distance_threshold

        self.algorithm = algorithm.lower()
        self.msg = Twist()
        # avoid cycles
        self.cycle_counter = 0
        self.cycle_threshold = cycle_threshold
        # Initialize speed parameters
        self.fast_lin_speed = fast_lin_speed
        self.mid_lin_speed = mid_lin_speed
        self.slow_lin_speed = slow_lin_speed
        self.fast_ang_speed = fast_ang_speed
        self.mid_ang_speed = mid_ang_speed
        self.slow_ang_speed = slow_ang_speed

        # Initialize sensor parameters
        self.close_distance_threshold = close_distance_threshold # meters
        self.far_distance_threshold = far_distance_threshold # meters
        self.curr_sensors_state_close = [0,0,0] # left1 front1, right1
        self.curr_sensors_state_far = [0,0,0] # left2 front2, right2

        # Initialize ros publishers, ros subscribers
        self.cmd_vel_pub = rospy.Publisher(pub_topic, Twist,
                          queue_size=100)
        self.laser_sub0 = rospy.Subscriber(sub_topic0, LaserScan,self.laserCallback_0, queue_size=100)
        self.laser_sub1 = rospy.Subscriber(sub_topic1, LaserScan,self.laserCallback_1, queue_size=100)
        self.laser_sub2 = rospy.Subscriber(sub_topic2, LaserScan,self.laserCallback_2, queue_size=100)


    def _change_distance_state(self, curr_ranges=None, sensor_index=None):
        '''
        changes state of a sensor given by index
        curr_ranges: ranges from sensor from callback
        sensor_index: (0, 1, 2 for left front and right)
        two distance thresholds are used  robot->->|->->|
        '''
        if curr_ranges is not None and sensor_index is not None:
            # if curr_ranges and sensor_index data are normal
            # change state according to distance
            ranges_array = np.array(curr_ranges)
            obstacle_distance = ranges_array.min()

            if 0 < obstacle_distance <= self.close_distance_threshold:
                # # sensor very close to obstacle (set close and far to 1)
                self.curr_sensors_state_close[sensor_index] = 1
                self.curr_sensors_state_far[sensor_index] = 1
            elif obstacle_distance <= self.far_distance_threshold:
                # sensor near to obstacle
                self.curr_sensors_state_close[sensor_index] = 0
                self.curr_sensors_state_far[sensor_index] = 1
            elif obstacle_distance > self.far_distance_threshold:
                # obstacles are too far
                self.curr_sensors_state_close[sensor_index] = 0
                self.curr_sensors_state_far[sensor_index] = 0
            else:
                # obstacle_distance_left < 0
                #rospy.logwarn
                rospy.loginfo("obstacle_distance: {}, of sensor: {}".format(obstacle_distance, sensor_index))
        else:
            # rospy.logerror
            rospy.loginfo("curr_ranges={}, sensor_index= {}".format(curr_ranges, sensor_index))


    def laserCallback_0(self, msgIn):
        '''left sensor callback
        changes state of left sensor '''
        ranges_left = np.array(msgIn.ranges)
        self._change_distance_state(curr_ranges=ranges_left, sensor_index=0)

    def laserCallback_1(self, msgIn):
        '''front sensor callback
        changes state of front sensor '''
        ranges_front = np.array(msgIn.ranges)
        self._change_distance_state(curr_ranges=ranges_front, sensor_index=1)

    def laserCallback_2(self, msgIn):
        '''right sensor callback
        changes state of right sensor '''
        ranges_right = np.array(msgIn.ranges)
        self._change_distance_state(curr_ranges=ranges_right, sensor_index=2)

    def turn_left(self, ang_speed=np.pi/4):
        '''speed in rad/s'''
        self.msg.linear.x = 0.0
        self.msg.angular.z = ang_speed
        self.cmd_vel_pub.publish(self.msg)

    def turn_right(self, ang_speed=np.pi/4):
        '''speed in rad/s'''
        self.msg.linear.x = 0.0
        self.msg.angular.z = -ang_speed
        self.cmd_vel_pub.publish(self.msg)
   
    def move_forward_left(self, lin_speed=1, ang_speed=np.pi/8):
        '''speed in m/s'''
        self.msg.linear.x = lin_speed
        self.msg.angular.z = ang_speed
        self.cmd_vel_pub.publish(self.msg)
        
    def move_forward_right(self, lin_speed=1, ang_speed=np.pi/8):
        '''speed in m/s'''
        self.msg.linear.x = lin_speed
        self.msg.angular.z = -ang_speed
        self.cmd_vel_pub.publish(self.msg)

    def move_forward(self, lin_speed=1):
        '''speed in m/s'''
        self.msg.linear.x = lin_speed
        self.msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.msg)

    def move_backward(self, speed=1):
        '''speed in m/s not needed yet'''
        self.msg.angular.z = -speed
        self.msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.msg)

    def follow_left_wall(self):
        
        '''following left wall'''
        # mb subscribers here
        # rospy.loginfo(
        #     "far state: {}, close state: {}".format(self.curr_sensors_state_far, self.curr_sensors_state_close))

        if self.cycle_counter >= self.cycle_threshold:
            self.cycle_counter = -self.cycle_threshold

        if not any(self.curr_sensors_state_far) and not any(self.curr_sensors_state_close):
            # no obstacles (like [0,0,0] [0,0,0])
            self.move_forward_left(lin_speed=self.fast_lin_speed, ang_speed=self.slow_ang_speed)
            self.cycle_counter+=1
            
        elif any(self.curr_sensors_state_far) and not any(self.curr_sensors_state_close):
            # obstacles far (like [0,1,0] [0,0,0])
            self.move_forward_left(lin_speed=self.mid_lin_speed, ang_speed=self.slow_ang_speed)
            self.cycle_counter+=1

        elif    self.curr_sensors_state_close == [1, 1, 0] or \
                self.curr_sensors_state_close == [1, 0, 0]:
            # wall on the left
            self.move_forward_right(lin_speed=self.mid_lin_speed, ang_speed=self.mid_ang_speed)
            # self.cycle_counter=0

        elif    self.curr_sensors_state_close == [0, 1, 1] or \
                self.curr_sensors_state_close == [0, 0, 1] or \
                self.curr_sensors_state_close == [0, 1, 0] or \
                self.curr_sensors_state_close == [1, 1, 1]:
            # wall on the right or in front or everywhere
            self.turn_right(ang_speed=self.fast_ang_speed)
            self.cycle_counter=0

        elif    self.curr_sensors_state_close == [1, 0, 1]:
            # wall on the right and on the left
            self.move_forward(lin_speed=self.slow_lin_speed)
            # self.cycle_counter=0

    def follow_right_wall(self):

        '''following left wall'''
        # mb subscribers here
        # rospy.loginfo(
        #     "far state: {}, close state: {}".format(self.curr_sensors_state_far, self.curr_sensors_state_close))

        if not any(self.curr_sensors_state_far) and not any(self.curr_sensors_state_close):
            # no obstacles (like [0,0,0] [0,0,0])
            self.move_forward_right(lin_speed=self.fast_lin_speed, ang_speed=self.slow_ang_speed)
            self.cycle_counter+=1

        elif any(self.curr_sensors_state_far) and not any(self.curr_sensors_state_close):
            # obstacles far (like [0,1,0] [0,0,0])
            self.move_forward_right(lin_speed=self.mid_lin_speed, ang_speed=self.slow_ang_speed)
            self.cycle_counter+=1

        elif    self.curr_sensors_state_close == [0, 1, 1] or \
                self.curr_sensors_state_close == [0, 0, 1]:
            # wall on the right
            self.move_forward_left(lin_speed=self.mid_lin_speed, ang_speed=self.mid_ang_speed)

        elif    self.curr_sensors_state_close == [1, 1, 0] or \
                self.curr_sensors_state_close == [1, 0, 0] or \
                self.curr_sensors_state_close == [0, 1, 0] or \
                self.curr_sensors_state_close == [1, 1, 1]:
            # wall on the left or in front or everywhere
            self.turn_left(ang_speed=self.fast_ang_speed)
            # twice threshold
            self.cycle_counter=-self.cycle_threshold

        elif    self.curr_sensors_state_close == [1, 0, 1]:
            # wall on the right and on the left
            self.move_forward(lin_speed=self.slow_lin_speed)

    def random_navigation(self):
        pass

def main():

    rospy.init_node('reactive_nav')

    commander = Navigator(**navigation_parameters)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        if 0 <= commander.cycle_counter <= commander.cycle_threshold:
            # increments in cycles
            rospy.loginfo("commander.cycle_counter: {}".format(commander.cycle_counter))
            commander.follow_left_wall()
        elif -commander.cycle_threshold <= commander.cycle_counter < 0:
            # decrements in cycles
            rospy.loginfo("commander.cycle_counter: {}".format(commander.cycle_counter))
            commander.follow_right_wall()
        else:
            rospy.loginfo("commander.cycle_counter: {}".format(commander.cycle_counter))

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass