#!/usr/bin/env python3


import threading
import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState



class Control:
    speed= 521
    kp=5
    kd=1
    prev_err=0
    n = 10
    def get_angle_from_mv(self):
        return 45
        
    def PD_control(self, angle):
        err=90-angle
        der=self.prev_err-err
        cont=self.kp*err-self.kd*der
        self.prev_err=err
        speed_L=min(max(self.speed+cont,0),1023)
        speed_R=min(max(self.speed-cont,0),1023)		
        return [speed_L, speed_R]
    
    def get_keyboard_input(self):
      while(True):
        inp = input("type something ")
        #print("key: " + inp)
        if(inp == "quit"):
          return
        self.n = int(inp)
		
def get_joint_state(cont):
    joints = JointState()
    joints.header.stamp = rospy.get_rostime()
    joints.name = ["left_wheel", "right_wheel"]
    joints.velocity = cont
    return joints

def talker(c):
    pub = rospy.Publisher('motor_control', JointState, queue_size=10)
    rospy.init_node('move_control', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cont = c.PD_control(c.n)
        joints = get_joint_state(cont)
        print(joints)
        pub.publish(joints)
        rate.sleep()



if __name__ == '__main__':
    try:
        c = Control()
        x = threading.Thread(target=c.get_keyboard_input)
        x.start()
        talker(c)

    except rospy.ROSInterruptException:
        pass


