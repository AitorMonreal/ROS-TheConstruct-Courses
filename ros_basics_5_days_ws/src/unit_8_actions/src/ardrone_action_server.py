#! /usr/bin/env python
import rospy
import time
import actionlib

from actionlib.msg import TestFeedback, TestResult, TestAction
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

class SquareClass(object):
    
  # create messages that are used to publish feedback/result
  _feedback = TestFeedback()
  _result   = TestResult()

  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("square_as", TestAction, self.goal_callback, False)
    self._as.start()
    
  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that performs the square of size 'goal'
    # and returns the time elapsed to the node that called the action server
    
    # helper variables
    r = rospy.Rate(1)                                            
    success = True
    
    # publish info to the console for the user
    rospy.loginfo('"square_as": Executing, moving in a square of size %i' % goal.goal)
    
    # starts performing the square
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    move = Twist() 
    side = 0
    pub_takeoff = rospy.Publisher('drone/takeoff', Empty, queue_size=1)
    while True: # repeat until break
        rospy.Rate(0.5).sleep() # wait a bit for the connection to establish
        connections = pub_takeoff.get_num_connections()
        if connections > 0:
            # pub_takeoff.publish(empty_message)
            pub_takeoff.publish()
            rospy.Rate(0.5).sleep()
            rospy.loginfo("Taken Off")
            break
        else:
            rospy.Rate(0.5).sleep() # if the connection hasn't been established, wait a bit longer

    for i in range(4):
      
      # check that preempt (cancelation) has not been requested by the action client
      if self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        # the following line, sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False
        # we end the calculation of the Fibonacci sequence
        break
      
      # Move straight, stop & turn, stop
      move.linear.x = 0.2
      pub.publish(move)
      rospy.Rate(1/(goal.goal*1.0)).sleep()
      move.linear.x = 0
      move.angular.z = 0.8  # This is what we change until we get the robot to perform a 90deg turn
      pub.publish(move)
      rospy.Rate(0.5).sleep()
      move.angular.z = 0
      pub.publish(move)
      r.sleep()

      
      # Increase the side we are at by 1, and publish as feedback
      side += 1
      self._feedback.feedback = side
      self._as.publish_feedback(self._feedback)

    pub_landing = rospy.Publisher('/drone/land', Empty, queue_size=1)
    pub_landing.publish()
    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    # If success, then we publish the final result
    # If not success, we do not publish anything in the result
    if success:
      self._result.result = rospy.get_time()
      rospy.loginfo('Succeeded performing the square of size %i in %i time' % ( goal.goal, self._result.result))
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('square')
  SquareClass()
  rospy.spin()