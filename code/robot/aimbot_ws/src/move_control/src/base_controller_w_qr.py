#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState

class Control:
    speed= 521
    kp=5
    kd=1
    prev_err=0
        
    def PD_control(self, angle, arrow):
        if angle.qr_code==1 && arrow==0
            speed_L=0
            speed_R=0
        elif angle.qr_code==1 && arrow==1    #turn left from arrowhead
            speed_L=30
            speed_R=621
        elif angle.qr_code==1 && arrow==2    #turn right from arrowhead
            speed_L=621
            speed_R=30
        elif angle.qr_code==1 && arrow==3    #Drive straight from arrowhead
            speed_L=521
            speed_R=521
        elif angle.qr_code==1                
            speed_L=0
            speed_R=0
        else
            err=90-angle.angle
            der=self.prev_err-err
            cont=self.kp*err-self.kd*der
            self.prev_err=err
            speed_L=min(max(self.speed+cont,0),1023)
            speed_R=min(max(self.speed-cont,0),1023)		
        return [speed_L, speed_R]
		
    def send_to_PD(self, data):
      i = Int32()
      i.data = data.data
      cont = self.PD_control(i.data)
      joint = get_joint_state(cont)
      print(joint)
      pub.publish(joint)




def get_joint_state(cont):
    joints = JointState()
    joints.header.stamp = rospy.get_rostime()
    joints.name = ["left wheel", "right_wheel"]
    joints.velocity = cont
    return joints



def listener(): 	# Set up subscriber to ros-node
    # Initialization
    base_sub = rospy.Subscriber("mv", Int32, c.send_to_PD)
    #ats = ApproximateTimeSynchronizer([base_sub], queue_size=2, slop=0.1)
   #ats.registerCallback(send_to_motors)
    rospy.spin()
    
def listener(): 	# Set up subscriber to ros-node
    # Initialization
    base_sub = rospy.Subscriber("Arrow", Int32, A.send_to_PD)
    #ats = ApproximateTimeSynchronizer([base_sub], queue_size=2, slop=0.1)
   #ats.registerCallback(send_to_motors)
    rospy.spin()
if __name__ == '__main__':
    c = Control()
    pub = rospy.Publisher('motor_control', JointState, queue_size=10)
    rospy.init_node('move_control', anonymous=True)
    rate = rospy.Rate(10)
    listener()

			
	








