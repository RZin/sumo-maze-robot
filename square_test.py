#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Pose2D, Quaternion, Twist, Vector3
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import Range
import tf

import numpy as np

from robot_driver import Driver

FW_speed = float(0.05)
TL_speed = float(np.pi/10.5)

driver_parameters = {
    'rate': 10,
}

sensor_params = {
    'radiation_type' : 1,# ir
    'field_of_view' : np.pi/12,
    'min_range' : 0.1,
    'max_range' : 0.8
}

sensor_names = {
    'left':'left_ir',
    'front':'front_ir',
    'right':'right_ir'
}

def normalize_angle(angle):
    return np.sin(angle)/np.cos(angle) # not sure


def move_by_ticks(robot, counter, counter_change_state):
    if 0 <= counter < counter_change_state:
        robot.move_forward(lin_speed=FW_speed/2)
    elif  counter_change_state <= counter < 2*counter_change_state:
        robot.move_forward(lin_speed=FW_speed)
    elif 2 * counter_change_state <= counter < 3 * counter_change_state:
        robot.turn_left(ang_speed=TL_speed)
    elif 3 * counter_change_state <= counter < 4 * counter_change_state:
        robot.move_forward(lin_speed=FW_speed)
    elif 4 * counter_change_state <= counter < 5 * counter_change_state:
        robot.turn_left(ang_speed=TL_speed)
    elif 5 * counter_change_state <= counter < 6 * counter_change_state:
        robot.move_forward(lin_speed=FW_speed)
    elif 6 * counter_change_state <= counter < 7 * counter_change_state:
        robot.turn_left(ang_speed=TL_speed)
    elif 7 * counter_change_state <= counter < 8 * counter_change_state:
        robot.move_forward(lin_speed=FW_speed*0.1)
    elif 8 * counter_change_state <= counter < 9 * counter_change_state:
        robot.turn_left(ang_speed=TL_speed)
    else:
        robot.stop_moving()

def movements_by_ticks(driver, counter, counter_change_state):
    rospy.init_node('robot_driver')

    driver = Driver(**driver_parameters)

    counter = 0
    counter_change_state = 20
    while not rospy.is_shutdown():

        # rospy.loginfo("random_navigation")
        driver.publish_odom()

        # move_by_ticks(driver, counter, counter_change_state)

        if  driver.left_distance_val <= 0.15:
            rospy.logwarn("left Collision risk!")

        elif driver.front_distance_val <= 0.15:
            rospy.logwarn("front Collision risk!")

        elif driver.right_distance_val <= 0.15:
            rospy.logwarn("right Collision risk!")

        counter += 1
        driver.rate.sleep()



def move_to(robot, goal_distance, linear_speed):
    assert robot.hasattr('x')
    assert robot.hasattr('y')
    assert robot.hasattr('move_forward')
    assert robot.hasattr('rate')

    # Keep track of the distance traveled
    x_start = robot.x
    y_start = robot.y
    distance = 0

    # Enter the loop to move along a side
    while distance < goal_distance and not rospy.is_shutdown():
        # move_forward and sleep 1 cycle
        robot.move_forward(linear_speed)
        robot.rate.sleep()

        # Recieving current position from callback
        # Compute the Euclidean distance from the start
        distance = sqrt(pow((robot.x - x_start), 2) +
                        pow((robot.y - y_start), 2))

    # Stop the robot
    driver.cmd_vel_pub.publish(Twist())
    rospy.sleep(1.0)
    return

def rotate_to(robot, goal_angle, angular_speed):
    assert robot.hasattr('theta')
    assert robot.hasattr('turn_left')
    assert robot.hasattr('rate')

    angular_tolerance = np.radians(rospy.get_param("~angular_tolerance", 2))  # degrees to radians

    # Track the last angle measured
    last_angle = robot.theta  #

    # Track how far we have turned
    turn_angle = 0

    # Begin the rotation
    while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
        # Publish the Twist message and sleep 1 cycle
        robot.turn_left(ang_speed=angular_speed)
        robot.rate.sleep()

        # Recieving current position from callback
        # Compute the amount of rotation since the last loop
        delta_angle = normalize_angle(robot.theta - last_angle) # not sure if we need to normalize

        turn_angle += delta_angle
        last_angle = robot.theta

    # Stop the robot
    driver.cmd_vel_pub.publish(Twist())
    rospy.sleep(1.0)
    return


def movements_by_odom(cycles_num=1):

    rospy.init_node('robot_driver')

    # Set the parameters for the target square
    goal_distance = rospy.get_param("~goal_distance", 1.0)  # meters
    goal_angle = rospy.get_param("~goal_angle", np.pi/2)  # radians
    linear_speed = rospy.get_param("~linear_speed", 0.5)  # meters per second
    angular_speed = rospy.get_param("~angular_speed", 0.7)  # radians per second


    driver = Driver(**driver_parameters)

    # Cycle through the four sides of the square
    for i in range(cycles_num*4):

        move_to(robot=driver, goal_distance=goal_distance, linear_speed=linear_speed)

        rotate_to(robot=driver, goal_angle=goal_angle, angular_speed=angular_speed)



def main():
    movements_by_odom(cycles_num=1)


if __name__ == '__main__':
    try:
        main()
        pass
    except rospy.ROSInterruptException:
        pass