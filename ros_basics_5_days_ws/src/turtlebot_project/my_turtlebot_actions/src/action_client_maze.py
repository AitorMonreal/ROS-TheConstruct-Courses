#!/usr/bin/env python

# First start the Action Server, then either publish a goal to the action (in this case, Empty: 
# 'rostopic pub /rec_odom_as/goal my_turtlebot_actions/record_odomActionGoal "{}"'), or run the 
# Action Client which also sends an Empty Goal to the Server.

import rospy
import time
import actionlib
from my_turtlebot_actions.msg import record_odomAction, record_odomResult, record_odomGoal, record_odomFeedback
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty

def call_server():
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

    # create the connection to the action server
    client = actionlib.SimpleActionClient('rec_odom_as', record_odomAction)
    # waits until the action server is up and running
    rospy.loginfo('Waiting for Action Server')
    client.wait_for_server()
    rospy.loginfo('Action Server Ready')

    # sends the goal to the action server, specifying which feedback function
    # to call when feedback received
    client.send_goal(Empty())
    rospy.loginfo('Goal sent')

    client.wait_for_result()
    rospy.loginfo('Waiting for Result')

    state_result = client.get_state()

    rospy.loginfo("[Result] State: "+str(state_result))
    if state_result == ERROR:
        rospy.logerr("Something went wrong in the Server Side")
    if state_result == WARN:
        rospy.logwarn("There is a warning in the Server Side")

    result = client.get_result()
    return result


if __name__ == '__main__':
    try:
        rospy.init_node('maze_action_client')
        result = call_server()
        print 'The result is:', result
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e