#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry

def calculate_distance(array):
    start_odom = array[0]
    end_odom = array[len(array)-1]

    x_position_0 = start_odom.position.x
    y_position_0 = start_odom.position.y

    x_position_end = end_odom.position.x
    y_position_end = end_odom.position.y

    vector_0 = [x_position_0, y_position_0]
    vector_end = [x_position_end, y_position_end]
    distance = math.sqrt(math.pow(x_position_end-x_position_0,2) + math.pow(y_position_end-y_position_0,2))
    
    return distance


def check_if_out_maze(goal_distance, result_odom_array):
    distance = calculate_distance(result_odom_array)
    rospy.loginfo("Distance Moved="+str(distance))
    if distance < goal_distance:
        return True
    
