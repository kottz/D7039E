#!/usr/bin/env python3

import threading
import rospy
from std_srvs.srv import *
from sensor_msgs.msg import JointState
from arm_control.srv import *
import time
rospy.init_node('robot_main')

qr_name = "0,0"
rospy.wait_for_service("reset_arm")
rospy.wait_for_service("follow_line")
rospy.wait_for_service("stop")
rospy.wait_for_service("pickup")
try:
    arm_reset = rospy.ServiceProxy("reset_arm",Empty)
    follow_line = rospy.ServiceProxy("follow_line", Empty)
    stop_robot = rospy.ServiceProxy("stop", Empty)
    pickup = rospy.ServiceProxy("pickup", Empty)
    ah_req = rospy.ServiceProxy("ah_req", ah_request)
    turn_left = rospy.ServiceProxy("turn_left", Empty)
    turn_right = rospy.ServiceProxy("turn_right", Empty)
    turn_back = rospy.ServiceProxy("turn_back", Empty)
    resp = arm_reset()
    rospy.logwarn("Arm RESET")
    time.sleep(2)
except rospy.ServiceException as e:
    print("error when calling arm_joints_srv: %s" %e)
    sys.exit(1)




def runner():
    arm_reset()
    pickup()
    follow_line()
    time.sleep(10)
    turn_right()
    
class RobotControl:
    last_qr = ""
    prev_qr = "0"
    qr_lock = threading.Lock()
    #what do we want?
    #This should call other services and in that way facilitate the entire routine

    #follow line until we reach a qr code. Then query arrowhead for next action
    def init_listeners(self):	
            qr_sub = rospy.Subscriber("mv_qr", JointState, self._qr_callback)
    
    def _qr_callback(self, data):
        with qr_lock:
            self.prev_qr = self.cur_qr
            self.cur_qr = data.name

    #quite ugly but will work for now
#    def wait_for_qr(self):
#        while True:
#            with self.qr_lock:
#                if self.cur_qr != self.prev_qr:
#                    return
#            time.sleep(1)
#
    def run(self):
        arm_reset()
        while True:
            ah_resp = ah_req(self.last_qr)
            
            rospy.logwarn(ah_resp.response)
            
            if ah_resp.response == "left":
                turn_left()
                follow_line()
                #time.sleep(10)
            elif ah_resp.response == "right":
                turn_right()
                follow_line()
                #time.sleep(10)
            elif ah_resp.response == "follow":
                follow_line()
                #time.sleep(10)
            elif ah_resp.response == "pickup":
                pickup()
            elif ah_resp.response == "back":
                turn_back()


rc = RobotControl()

rc.init_listeners()
rc.run()

rospy.spin()















