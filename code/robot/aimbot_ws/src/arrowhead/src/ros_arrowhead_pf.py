#!/usr/bin/env python3



import rospy
import threading


from arm_control.srv import *

class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Given two adjacent squares. Calculate direction of other relative self.
    # 0 = forward, 1 = right, 2 = back, 3 = left
    def adj_dir(self, other):
        if self.x == other.x:
            if self.y > other.y:
                #Go forward
                return 0
            else:
                #Go back
                return 2
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
            
#Okej vad vill jag ha...
#Funktion som tar in nuvarande och nästa qr och from_dir. Skickar ut left, right, forward eller back            
            

# In this example a = (0,0), b = (0,1), c = (1,0), d = (1,1)

# This is an adjecency matrix for the small four corner test setup
graph=  [#a,b,c,d
		 [0,3,1,0],  #a
		 [3,0,0,0],  #b
		 [1,0,0,3],  #c
		 [0,0,3,0],  #d
]

nodes=['(0,0)','(0,1)','(1,0)','(1,1)']
a=input('Source Destination = ')
b=input('Destination = ')
res = pathfind(a,b)
print(res)





def get_keyboard_input():
      while(True):
        inp = input("type something ")
        if inp == "follow":
            pass
        elif inp == "forward":
            pass
        elif inp == "left":
            pass
        elif inp == "right":
            pass
        elif inp == "back":
            pass
        elif inp == "pickup":
            pass
        print("key: " + inp)
        if(inp == "quit"):
            return
        self.n = int(inp)

def arrowhead_spoof(request):
    qr_data = request
    print(qr_data)
    inp = input("arrowhead input requested. What to do? ")
    # sry for ugly
    print("key: " + inp)
    return inp
    
cur_pos = None

def pathfind_and_follow():
    inp = input("arrowhead input requested. What to do? \n[pathfind | follow | forward | left | right | back | pickup]")


def go_to_goal(goal):
    # If we don't know where we are. Start to follow line until we find a QR with position data
    qr = None
    if cur_pos == None:
        qr = follow_line() #This service should just return the QR data.
    
    while(True):
        path = find_path(cur_pos, goal)

        # If the length is one we have arrived
        if len(path) == 1:
            return
        from_dir = int(qr.effort[0])
        dir = direction_to_turn(cur_pos, path[1], from_dir)

        if dir == 0:
            drive_forward()
        elif dir == 1:
            turn_right()
        elif dir == 2:
            turn_back()
        elif dir == 3:
            turn_left()
        qr = follow_line()
        cur_pos = qr.name # Update current position


if __name__ == '__main__':
    rospy.init_node('arrowhead', anonymous=True)

    x = threading.Thread(target=get_keyboard_input)
    x.start()
    x.join()
    rospy.Service('ah_req', ah_request, arrowhead_spoof)
    rospy.spin()
