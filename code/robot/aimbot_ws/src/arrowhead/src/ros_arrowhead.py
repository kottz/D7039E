#!/usr/bin/env python3
import rospy
import threading
from arm_control.srv import *

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
    


if __name__ == '__main__':
    rospy.init_node('arrowhead', anonymous=True)

    #x = threading.Thread(target=get_keyboard_input)
    #x.start()
    #x.join()
    rospy.Service('ah_req', ah_request, arrowhead_spoof)
    rospy.spin()
