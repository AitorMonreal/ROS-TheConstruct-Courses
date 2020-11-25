#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from laser_obstacle_subscriber import LaserSubscriber

class ObstacleService(object):
    def __init__(self):
        self._srv = rospy.Service('/obstacle', Trigger, self.obstacle_cb)
        self._lasersubscriber = LaserSubscriber()
        self._dict = {"front":False, "left":False, "right":False, "diff":0.0}
    
    def obstacle_cb(self, request):
        self._dict = self._lasersubscriber.obstacle_direction()
        print self._dict
        response = TriggerResponse()
        response.success = self.detect_obstacle()
        response.message = self.next_direction()

        return response
    
    def detect_obstacle(self):
        if True in self._dict.values():
            return True
        else:
            return False
    
    def next_direction(self):
        if not self._dict['front']:
            return 'forwards'
        else:
            if not self._dict['left'] and not self._dict['right']:
                if self._dict['diff'] > 0:
                    return 'left'
                else:
                    return 'right'
            elif self._dict['left'] and not self._dict['right']:
                return 'right'
            elif self._dict['right'] and not self._dict['left']:
                return 'left'
            else:
                return 'backwards'
    
    def keep_turning(self):
        

if __name__ == '__main__':
    rospy.init_node('obstacle_service_server')
    obstacleservice = ObstacleService()
    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        rospy.spin() # mantain the service open.