#!/usr/bin/env python

import rospy
import sys
import time
import actionlib
from motion_publisher import MotionPublisher
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Empty
from my_turtlebot_actions.msg import record_odomAction, record_odomResult, record_odomGoal, record_odomFeedback

class ServiceClient(object):
    def __init__(self):
        self._service = rospy.ServiceProxy('/obstacle', Trigger)
        self._service_object = TriggerRequest()
        self._result = None
    def next_direction(self):
        rospy.wait_for_service('/obstacle')
        self._result = self._service(self._service_object)
        if self._result.success:
            rospy.logwarn("Success == " + str(self._result.success)) # print the result given by the service called
            rospy.logwarn("Direction To Go == " + str(self._result.message)) # print the result given by the service called
        else:
            rospy.loginfo("Success == " + str(self._result.success)) # print the result given by the service called
            rospy.loginfo("Direction To Go == " + str(self._result.message)) # print the result given by the service called
        
        return self._result.message

class RobotMotion(object):

    def __init__(self):
        self._motionpublisher = MotionPublisher()
        self._serviceclient = ServiceClient()

    def exit_maze(self):
        ###
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

        state_result = client.get_state()
        while state_result < DONE:
            rate = rospy.Rate(2)
            direction = self._serviceclient.next_direction()
            self._motionpublisher.move_robot(direction)
            if direction == 'left' or direction == 'right':
                rospy.Rate(2).sleep()
            else:
                rate.sleep()
            state_result = client.get_state()
            rospy.loginfo("state_result: "+str(state_result))

        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == WARN:
            rospy.logwarn("There is a warning in the Server Side")

        rospy.loginfo("--- Finished ---")
        self._motionpublisher.move_robot("stop")
        rospy.is_shutdown()

        result = client.get_result()
        return result
        ###



if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_main_node')
        robotmotion = RobotMotion()
        while not rospy.is_shutdown(): 
            result = robotmotion.exit_maze()
            print 'The result is:', result
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print 'Something went wrong:', e
        rospy.is_shutdown()
    
