#! /usr/bin/env python
import rospy
import time
import actionlib

from actionlib.msg import TestFeedback, TestResult, TestAction
#from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction

class StartStopClass(object):
  rospy.loginfo("Hello")
  # create messages that are used to publish feedback/result
  _feedback = CustomActionMsgFeedback()
  #_feedback = TestFeedback()


  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
    # self._as = actionlib.SimpleActionServer("square_as", TestAction, self.goal_callback, False)
    self._as.start()
    
  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that performs the square of size 'goal'
    # and returns the time elapsed to the node that called the action server
    
    # helper variables
    r = rospy.Rate(1)                                            
    success = True
    rospy.loginfo("Hello1")
    # publish info to the console for the user
    rospy.loginfo('"startstop_as": Executing, performing the action of %s' % goal.goal)
    
    # starts performing the action - either take off or land
    print 'drone/' + str(goal.goal)
    pub_action = rospy.Publisher('drone/' + str(goal.goal), Empty, queue_size=1)
    # pub_action = rospy.Publisher('drone/takeoff', Empty, queue_size=1)
    rospy.loginfo("Hello2")
    while True: # repeat until break
        self._feedback.feedback = goal  # set the feedback
        self._as.publish_feedback(self._feedback)  # publish the feedback
        rospy.Rate(0.5).sleep() # wait a bit for the connection to establish
        connections = pub_action.get_num_connections()
        rospy.loginfo("Hello3")
        if connections > 0:
            rospy.loginfo("Connected to Publisher")
            # pub_takeoff.publish(empty_message)
            pub_action.publish()
            rospy.Rate(0.5).sleep()
            rospy.loginfo("Published")
            break
        else:
            rospy.loginfo("Hello4")
            rospy.Rate(0.5).sleep() # if the connection hasn't been established, wait a bit longer

    # Even after we have compledted the action we cotninue publishing the feedback until the user breaks
    # while not rospy.is_shutdown():
    rospy.loginfo("Hello5")
    self._feedback.feedback = goal
    self._as.publish_feedback(self._feedback)
    r.sleep()

    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    # If success, then we publish the final result
    # If not success, we do not publish anything in the result
    if success:
      rospy.loginfo('Succeeded performing the action of %s' % goal.goal)
      self._as.set_succeeded()
      
if __name__ == '__main__':
  rospy.init_node('startstop')
  print "Hello"
  StartStopClass()
  rospy.spin()
