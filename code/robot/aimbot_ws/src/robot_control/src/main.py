#!/usr/bin/env python3

import threading
import rospy
from std_srvs.srv import *
from sensor_msgs.msg import JointState
from arm_control.srv import *
from motor_control.srv import *
import time
rospy.init_node('robot_main')

qr_name = "0,0"
rospy.wait_for_service("reset_arm")
print("efter waits")
rospy.wait_for_service("follow_line")
print("efter waits")
rospy.wait_for_service("stop")
rospy.wait_for_service("pickup")
print("efter waits")
try:
    arm_reset = rospy.ServiceProxy("reset_arm",Empty)
    follow_line = rospy.ServiceProxy("follow_line", Empty)
    drive_forward = rospy.ServiceProxy("drive_forward", Empty)
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


class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    # Input (0,0)
    def from_string(string):
        return Coordinate(int(string[1]), int(string[3]))
    
    def __str__(self):
        return "({}, {})".format(str(self.x), str(self.y))
    def __repr__(self):
        return "({}, {})".format(str(self.x), str(self.y))
        
    #def __eq__(self, other):
    #    return self.x == other.x and self.y == other.y
        
    # Given two adjacent squares. Calculate direction of other relative self.
    # 0 = forward, 1 = right, 2 = back, 3 = left
    def adj_dir(self, other):
        if self.x == other.x:
            if self.y > other.y:
                #Go forward
                return 2
            else:
                #Go back
                return 0
        if self.y == other.y:
            if self.x > other.x:
                #Go right
                return 1
            else:
                #Go left
                return 3



# Calculates which way to turn given two adjacent squares and a from_dir. 
def direction_to_turn(src, dst, from_dir):
    next_square_dir = src.adj_dir(dst)
    # Offset with the direction we are coming from (This will probably need some adjusting to get the directions right)
    direction = (next_square_dir + from_dir + 2) % 4
    return direction

def find_path(src, dst):
    p=[[src]]
    while p:
        x = p.pop(0)
        j = nodes.index(x[-1])
        if nodes[j] == dst:
            return x
        for i, e in enumerate(nodes):
            if graph[j][i] and e not in x:
                p.append(x + [e])
    return None
                     
           
# In this example a = (0,0), b = (0,1), c = (1,0), d = (1,1)

# This is an adjecency matrix for the small four corner test setup
graph=  [#a,b,c,d
         [0,3,1,0],  #a
         [3,0,0,0],  #b
         [1,0,0,3],  #c
         [0,0,3,0],  #d
]
#nodes=[]
#for x in range(0,2):
#    for y in range(0,2):
#        print(x, y)
#        nodes.append(Coordinate(x,y))

#print("testteset")
#print(nodes)
nodes=['{0,0}','{0,1}','{1,0}','{1,1}']
#a=input('Source Destination = ')
#b=input('Destination = ')

#res = pathfind(nodes[int(a)],nodes[int(b)])
#print(res)


def runner():
    arm_reset()
    pickup()
    follow_line()
    time.sleep(10)
    turn_right()
    
class RobotControl:
    last_qr = ""
    cur_pos = None
    prev_pos = None
    last_from_dir = -1
    prev_qr = "0"
    qr_lock = threading.Lock()
    #what do we want?
    #This should call other services and in that way facilitate the entire routine

    #follow line until we reach a qr code. Then query arrowhead for next action
    def init_listeners(self):
            qr_sub = rospy.Subscriber("mv_qr", JointState, self._qr_callback)
    
    def _qr_callback(self, data):
        with self.qr_lock:
            self.prev_qr = self.last_qr
            self.last_qr = data.name[0]
            self.last_from_dir = int(data.effort[0])
            self.cur_pos = self.last_qr

    #quite ugly but will work for now
#    def wait_for_qr(self):
#        while True:
#            with self.qr_lock:
#                if self.cur_qr != self.prev_qr:
#                    return
#            time.sleep(1)
#
    def go_to_goal(self, goal):
        # If we don't know where we are. Start to follow line until we find a QR with position data
        qr = None
        if self.cur_pos == None:
            follow_line() #This service should just return the QR data.
            while(self.cur_pos == self.prev_pos):
                time.sleep(1)
            # Qr format {0,1}
            time.sleep(2)
            with self.qr_lock:
                qr = self.last_qr
            print("last_qr: {}".format(qr))
            self.cur_pos = qr

        while(True):
            path = find_path(self.cur_pos, goal)
            print("path: {}, self.cur_pos: {}, goal: {}".format(str(path), str(self.cur_pos), str(goal)))
            
            # If the length is one we have arrived
            if path == None or len(path) == 1:
                print("no path or arrived at goal. returning")
                return
                
            direction = direction_to_turn(Coordinate.from_string(self.cur_pos), Coordinate.from_string(path[1]), self.last_from_dir)
            print("from_dir: {}, direction to turn: {}".format(str(self.last_from_dir), str(direction)))
            if direction == 0:
                drive_forward()
            elif direction == 1:
                turn_right()
            elif direction == 2:
                turn_back()
            elif direction == 3:
                turn_left()
                
            follow_line()
            #Block until we have reached a new QR code
            while(self.cur_pos == self.prev_pos):
                time.sleep(1)
                
            self.prev_pos = self.cur_pos
            with self.qr_lock:
                print("last_qr: {}".format(self.last_qr))
                self.cur_pos = self.last_qr


    def run(self):
        arm_reset()
        while True:
            ah_resp = ah_req(self.last_qr)
            
            rospy.logwarn(ah_resp.response)
            
            if ah_resp.response == "left":
                turn_left()
                #follow_line()
                #time.sleep(10)
            elif ah_resp.response == "right":
                turn_right()
                #follow_line()
                #time.sleep(10)
            elif ah_resp.response == "follow":
                follow_line()
                #time.sleep(10)
            elif ah_resp.response == "pickup":
                pickup()
            elif ah_resp.response == "back":
                turn_back()
            #Expect input of type "goal x y" (Here we should really sanitize input)
            elif ah_resp.response[:4] == "goal":
                goal_arr = ah_resp.response.split(" ")
                goal = goal_arr[1] #Should be of format {1,1} (same as data on qr)
                self.go_to_goal(goal)



rc = RobotControl()

rc.init_listeners()
rc.run()

rospy.spin()















