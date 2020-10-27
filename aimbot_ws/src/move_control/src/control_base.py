#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState

class Control:
    speed= 521
    kp=5
    kd=1
    prev_err=0

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
		
def get_joint_state(cont):
    joints = JointState()
    joints.header.stamp = rospy.get_rostime()
    joints.name = ["left wheel", "right_wheel"]
    joints.velocity = cont
    return joints

def talker():
    c = Control()
    pub = rospy.Publisher('motor_control', JointState, queue_size=10)
    rospy.init_node('move_control', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        a = c.get_angle_from_mv()
        cont = c.PD_control(a)
        joints = get_joint_state(cont)
        print("sent")
        #joints.header.stamp = rospy.Time.now()
        pub.publish(joints)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

			
	








