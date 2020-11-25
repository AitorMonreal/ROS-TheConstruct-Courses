#!/usr/bin/env python

# Service to tell us if Sphero has crashed and in what direction to move in
# Service Request: Empty - no data needed
# Service Response: boolean - True or False crash
#                   string - In what direction to move now that we have crashed

import rospy
# from my_sphero_topics.src.topic_subscriber_crash_imu import IMUTopicReader
from std_srvs.srv import Trigger, TriggerResponse
from topic_subscriber_crash_imu import IMUTopicReader

class CrashDirectionService(object):
    
    def __init__(self, srv_name='/crash_service'):
        self._srv_name = srv_name
        self._service = rospy.Service(self._srv_name, Trigger, self.srv_callback)  # If we write 'self.srv_callback()', we will get an error saying that we're giving 1 attribute (self), when 2 are expected
        self._imu_reader = IMUTopicReader()
        self._dict = {"front":False, "left":False, "right":False, "back":False}
    '''
    def srv_callback(self, request):  # If we don't have 'request' here we will get an error saying that self._dict hasn't been defined for RobotCrash object
        self._dict = self._imu_reader.crash_direction_detection()
        print self._dict.values()
        if True in self._dict.values():
            rospy.logerr('HELLO')
            self._result.success = True
             
            key_list = list(self._dict.keys()) 
            val_list = list(self._dict.values()) 
            crash_direction = key_list[val_list.index(True)]
            rospy.logerr(crash_direction)
    '''
    def srv_callback(self, request):  
        # If we don't have 'request' here we will get an error saying that self._dict hasn't been defined for RobotCrash object
        self._dict = self._imu_reader.crash_direction_detection()
        response = TriggerResponse()
        response.success = self.has_crashed()
        response.message = self.direction_to_move_in()

        return response

    '''
        if True in self._dict.values():
            rospy.logerr('HELLO')
            self._result.success = True
             
            key_list = list(self._dict.keys()) 
            val_list = list(self._dict.values()) 
            crash_direction = key_list[val_list.index(True)]
            rospy.logerr(crash_direction)
    '''
    def has_crashed(self):
        if True in self._dict.values():
            return True
        else:
            return False

    def direction_to_move_in(self): 
        # If we haven't collided on the front and therefore can move forwards, we do. Else, if we can't move forwards but we can move backwards, we do.
        # Else, if we can't move forwards, nor backwards, but we can move to the left, we do. Else, we move to the right.
        if not self._dict['front']:
            return 'forwards'
        else:
            if not self._dict['back']:
                return 'backwards'
            else:
                if not self._dict['left']:
                    return 'left'
                else:
                    if not self._dict['right']:
                        return 'right'
                    else:
                        return 'stop'
    '''
        if True not in self._dict.values():
            return 'forwards'
        else:
            key_list = list(self._dict.keys()) 
            val_list = list(self._dict.values()) 
            crash_dir = key_list[val_list.index(True)]
            # rospy.logerr(crash_direction)
            if crash_dir == 'forwards':
                return
    '''
        # Tells if there has been a crash - True or False
        # Tells in which direction to move next

if __name__ == '__main__':
    rospy.init_node('crash_service_server')
    crash_service = CrashDirectionService()
    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rospy.spin() # mantain the service open.