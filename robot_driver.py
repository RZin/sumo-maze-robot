#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion, Twist, Vector3
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Range
import tf

import numpy as np
import random

FW_speed = float(0.01)
TL_speed = float(np.pi/50)

driver_parameters = {
    'rate': 10
}

sensor_params = {
    'radiation_type' : 1,# ir
    'field_of_view' : 0.0349,
    'min_range' : 0.1,
    'max_range' : 0.8
}

sensor_names = {
    'left':'left_ir',
    'front':'front_ir',
    'right':'right_ir'
}

# def range_to_meters(IR_raw):
#     ''' gets in mm returns in m'''
#     return IR_raw

class Driver:

    def __init__(self, rate = 10):

        self.msg = Twist()

        # position data
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0
        self.rate = rate

        # distance data
        self.left_distance_val = None
        self.front_distance_val = None
        self.right_distance_val = None

        # Initialize ros publishers, ros subscribers
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.pose_sub = rospy.Subscriber('/pose', Pose2D, self.pose_cb, queue_size=100)
        self.odom_pub = rospy.Publisher('/odom', Odometry,
                                           queue_size=100)

        # sensors publishers subscribers
        self.left_distance_sub = rospy.Subscriber('/left_distance', Float32, self.left_distance_cb, queue_size=100)
        self.front_distance_sub = rospy.Subscriber('/front_distance', Float32, self.front_distance_cb, queue_size=100)
        self.right_distance_sub = rospy.Subscriber('/right_distance', Float32, self.right_distance_cb, queue_size=100)

        self.left_distance_pub = rospy.Publisher('/ir_left_sensor', Range, queue_size=100)
        self.front_distance_pub = rospy.Publisher('/ir_front_sensor', Range, queue_size=100)
        self.right_distance_pub = rospy.Publisher('/ir_right_sensor', Range, queue_size=100)

        # sensors velocities
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist,
                          queue_size=100)

    def left_distance_cb(self, MsgIn):
        '''creates range with params predefined on top and publishes'''
        self.left_distance_val = MsgIn.data

        range = Range(**sensor_params)
        range.header.frame_id = sensor_names['left']
        range.header.stamp = rospy.Time.now()
        range.range = MsgIn.data
        self.left_distance_pub.publish(range)


    def front_distance_cb(self, MsgIn):
        '''creates range with params predefined on top and publishes'''
        self.front_distance_val = MsgIn.data
        range = Range(**sensor_params)
        range.header.frame_id = sensor_names['front']
        range.header.stamp = rospy.Time.now()
        range.range = MsgIn.data
        self.front_distance_pub.publish(range)


    def right_distance_cb(self, MsgIn):
        '''creates range with params predefined on top and publishes'''
        self.right_distance_val = MsgIn.data
        range = Range(**sensor_params)
        range.header.frame_id = sensor_names['right']
        range.header.stamp = rospy.Time.now()
        range.range = MsgIn.data
        self.right_distance_pub.publish(range)

    def pose_cb(self, msgIn):

        '''Gets position and updates vars'''

        # update velocities
        self.v = (msgIn.x - self.x)*self.rate/np.cos(self.theta)
        # self.v = (msgIn.y - self.y)*self.rate/np.sin(self.theta)
        self.w = (msgIn.theta - self.theta)*self.rate

        # update pose
        self.x = msgIn.x
        self.y = msgIn.y
        self.theta = msgIn.theta

    def publish_odom(self):
        current_time = rospy.Time.now()

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            translation = (self.x, self.y, 0),
            rotation = odom_quat,
            time = current_time,
            child = "base_link",
            parent = "odom"
        )

        # next, create the odometry message and fill with data
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.v, 0, 0), Vector3(0, 0, self.w))

        # publish the message
        self.odom_pub.publish(odom)

    def turn_left(self, ang_speed=np.pi / 4):
        '''speed in rad/s'''
        self.msg.linear.x = 0.0
        self.msg.angular.z = ang_speed
        self.cmd_vel_pub.publish(self.msg)

    def turn_right(self, ang_speed=np.pi / 4):
        '''speed in rad/s'''
        self.msg.linear.x = 0.0
        self.msg.angular.z = -ang_speed
        self.cmd_vel_pub.publish(self.msg)

    def move_forward_left(self, lin_speed=1, ang_speed=np.pi / 8):
        '''speed in m/s'''
        self.msg.linear.x = lin_speed
        self.msg.angular.z = ang_speed
        self.cmd_vel_pub.publish(self.msg)

    def move_forward_right(self, lin_speed=1, ang_speed=np.pi / 8):
        '''speed in m/s'''
        self.msg.linear.x = lin_speed
        self.msg.angular.z = -ang_speed
        self.cmd_vel_pub.publish(self.msg)

    def move_forward(self, lin_speed=1):
        '''speed in m/s'''
        self.msg.linear.x = lin_speed
        self.msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.msg)

    def move_backward(self, lin_speed=1):
        '''speed in m/s not needed yet'''
        self.msg.linear.x = -lin_speed
        self.msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.msg)

    def stop_moving(self):
        '''speed in m/s not needed yet'''
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.msg)

    def set_buzzer(self):
        # set params
        rospy.wait_for_service('/clear')

        # rospy.set_param('/', random.randint(0, 255))

        clearClient = rospy.ServiceProxy('/clear', Empty)
        clearClient()


def test_movements_v1(driver, counter, counter_change_state):

    while 0 <= driver.x < 0.5:
        driver.move_forward(lin_speed=FW_speed)
    while 0 <= driver.theta < np.pi/2:
        driver.turn_left(ang_speed=TL_speed)
    while 0 <= driver.y < 0.5:
        driver.move_forward(lin_speed=FW_speed)
    while np.pi/2 <= driver.theta < np.pi:
        driver.turn_left(ang_speed=TL_speed)
    while 0 <= driver.x < 0.5:
        driver.move_forward(lin_speed=FW_speed)

    driver.stop_moving()


def test_movements(driver, counter, counter_change_state):

    if 0 <= counter < counter_change_state:
        driver.move_forward(lin_speed=FW_speed/3)
    elif  counter_change_state <= counter < 2*counter_change_state:
        driver.move_forward(lin_speed=FW_speed)
    elif 2 * counter_change_state <= counter < 3 * counter_change_state:
        driver.turn_left(ang_speed=TL_speed)
    elif 3 * counter_change_state <= counter < 4 * counter_change_state:
        driver.move_forward(lin_speed=FW_speed)
    elif 4 * counter_change_state <= counter < 5 * counter_change_state:
        driver.turn_left(ang_speed=TL_speed)
    elif 5 * counter_change_state <= counter < 6 * counter_change_state:
        driver.move_forward(lin_speed=FW_speed)
    elif 6 * counter_change_state <= counter < 7 * counter_change_state:
        driver.turn_left(ang_speed=TL_speed)
    elif 7 * counter_change_state <= counter < 8 * counter_change_state:
        driver.move_forward(lin_speed=FW_speed/2)
    elif 8 * counter_change_state <= counter < 9 * counter_change_state:
        driver.turn_left(ang_speed=TL_speed)
    # elif 9 * counter_change_state <= counter < 10 * counter_change_state:
    #     driver.move_forward(lin_speed=FW_speed)
    # elif 10 * counter_change_state <= counter < 11 * counter_change_state:
    #     driver.turn_left(ang_speed=TL_speed)
    # elif 11 * counter_change_state <= counter < 12 * counter_change_state:
    #     driver.move_forward(lin_speed=FW_speed)
    # elif 12 * counter_change_state <= counter < 13 * counter_change_state:
    #     driver.turn_left(ang_speed=TL_speed)
    else:
        driver.stop_moving()


def main():
    rospy.init_node('robot_driver')

    driver = Driver(**driver_parameters)

    counter = 0
    counter_change_state = 40
    rate = rospy.Rate(driver.rate)
    while not rospy.is_shutdown():

        # commander.random_navigation()
        # rospy.loginfo("random_navigation")
        driver.publish_odom()

        # test_movements(driver, counter, counter_change_state)

        rate.sleep()
        counter += 1

        rospy.loginfo("counter".format(str(counter)))

if __name__ == '__main__':
    try:
        # main()
        pass
    except rospy.ROSInterruptException:
        pass
