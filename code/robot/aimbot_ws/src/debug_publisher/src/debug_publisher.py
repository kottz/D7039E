#!/usr/bin/env python3


import threading
import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState



class Control:
    n = 0
    def get_keyboard_input(self):
      while(True):
        inp = input("type something ")
        #print("key: " + inp)
        if(inp == "quit"):
          return
        self.n = int(inp)


def talker(c):
    pub = rospy.Publisher('mv', Int32, queue_size=10)
    rospy.init_node('mv', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cont = c.n
        pub.publish(cont)
        rate.sleep()

if __name__ == '__main__':
    try:
        c = Control()
        x = threading.Thread(target=c.get_keyboard_input)
        x.start()
        talker(c)

    except rospy.ROSInterruptException:
        pass


