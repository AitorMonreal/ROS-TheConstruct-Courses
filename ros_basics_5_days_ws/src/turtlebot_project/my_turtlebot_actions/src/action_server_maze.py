#!/usr/bin/env python

import rospy
import actionlib
from odom_subscriber import OdomSubscriber
from my_turtlebot_actions.msg import record_odomResult, record_odomAction
from nav_msgs.msg import Odometry
from odometry_analysis import check_if_out_maze

# when the action is called, it starts to save Odometry data, checking if the robot has exited the maze or not
# also, if more than 'self._seconds_recording' have passed, we exit the action and set success to False

goal = rospy.get_param('goal_distance')
seconds = rospy.get_param('seconds_recording')

class OdomAction(object):

    def __init__(self, goal_distance=float(goal), seconds_recording=int(seconds)):
        # Creates the action server
        self._as = actionlib.SimpleActionServer("rec_odom_as", record_odomAction, self.execute_cb, False)
        self._as.start()
        # Create an object that reads from the topic Odom
        self._odom_sub_object = OdomSubscriber()
        # Create messages that are used to publish result
        self._result = record_odomResult()
        self._goal_distance = goal_distance
        self._seconds_recording = seconds_recording
    
    def execute_cb(self, goal):
        # this callback is called when the action server is called.
        rate = rospy.Rate(1)
        success = True

        self._result.result_odom_array.append(self._odom_sub_object.get_odomdata())

        for i in range(self._seconds_recording):
            rospy.loginfo("Recording Odom index="+str(i))
            #First check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the action loop
                break

            else:
                if not self.reached_distance_goal():
                    rospy.loginfo('Reading Odometry...')
                    self._result.result_odom_array.append(self._odom_sub_object.get_odomdata())
                else:
                    rospy.logwarn('Reached distance Goal')
                    # we end the action loop
                    break
                rate.sleep()

        # If the time has passed, exit the action
        if i == self._seconds_recording-1:
            rospy.loginfo('Time out, goal has been cancelled')
            self._as.set_preempted()
            success = False

        # If successful we publish the final result
        if success == True:
            self._as.set_succeeded(self._result)

        self.clean_variables()
    
    def clean_variables(self):
        """
        Cleans variables for the next call
        """
        self._result = record_odomResult()

    def reached_distance_goal(self):
        """
        Returns True if the distance moved from the first instance of recording until now has reached the self._goal_distance
        """
        return check_if_out_maze(self._goal_distance, self._result.result_odom_array)

if __name__ == '__main__':
    rospy.init_node('maze_action_server')
    while not rospy.is_shutdown():
        OdomAction()
        rospy.spin()