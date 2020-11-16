#!/usr/bin/env python3

import threading
import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState
import time

class PD:
    speed= 521
    kp=5
    kd=1
    prev_err=0
        
    def speed_from_angle(self, angle):
        err=90-angle
        der=self.prev_err-err
        cont=self.kp*err-self.kd*der
        self.prev_err=err
        speed_L=min(max(self.speed+cont,0),1023)
        speed_R=min(max(self.speed-cont,0),1023)		
        return [speed_L, speed_R]
			
class MoveControl:
    angle = 90
    follow = True
    qr_processing = False
    prev_qr = None



    def __init__(self):
        self.pd = PD()
        self.motor_publisher = rospy.Publisher('motor_control', JointState, queue_size=1)
        self.lock = threading.Lock()
    
        self.angle_lock = threading.Lock()
        self.qr_lock = threading.Lock()

    #Speed should be a list of two ints. [left speed, right speed]
    def ros_message_from_speed(self, speed):
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.name = ["left_wheel", "right_wheel"]
        msg.velocity = speed
        return msg
    # Should have a listener for angle which just updates the current angle on incoming messages, check if running 
    def start_listeners(self):
        angle_sub = rospy.Subscriber("mv", Int32, self._angle_callback)
        qr_sub = rospy.Subscriber("mv_qr", JointState, self._qr_callback)
        rospy.spin()

    def _angle_callback(self, angle):
        angle = angle.data
        #If angle hasn't changed, then we don't have to update anything
        if angle == self.angle:
            return

        with self.angle_lock:
            self.angle = angle

        with self.lock:
            follow = self.follow

        if follow:
            speed = self.pd.speed_from_angle(angle)
            speed_msg = self.ros_message_from_speed(speed)
            self.motor_publisher.publish(speed_msg)


    def _qr_callback(self, qr):
        #Only run this procedure once per intersection. 
        #print("In QR_callback") 
        data = qr.name[0]
        x = qr.position[0]
        y = qr.position[1]
        from_dir = int(qr.effort[0])

        if data == self.prev_qr:
            return
        self.prev_qr = data
        #outside_goal_rect = not ((0.2 < x < 0.8) and (0.2 < x < 0.8))

        if self.qr_processing: #or outside_goal_rect:
            return
        self.qr_processing = True

        with self.qr_lock:
            self.qr = qr




        #Turn off line following while we do the qr processing
        with self.lock:
            self.follow = False

        stop_msg = self.ros_message_from_speed([0,0])
        self.motor_publisher.publish(stop_msg) #stops the robot
        print("qr found in goal rect, stopping robot")
        time.sleep(0.5)

        
        #Blocking call waiting for a new move instruction
        
        #Here we then want to run a procedure to turn left/right/forward etc. For debug purposes we just try to turn left
        self.turn_left()

        #Start the line following again and turn off qr_processing mode
        with self.lock:
            self.follow = True
        self.qr_processing = False



    def turn_left(self):
        self._drive([621, 30], 4)

    def turn_right(self):
        self._drive([621, 30], 2)

    def drive_forward(self):
        self._drive([621, 621], 2)

    #method to turn around is also needed

    #Will set the motors to a certain speed for x sec
    def _drive(self, speed, time_to_drive):
        #Set motor to speed
        speed_msg = self.ros_message_from_speed(speed)
        self.motor_publisher.publish(speed_msg)

        #Let it turn for time_to_drive seconds
        time.sleep(time_to_drive)


        #Then stop again
        speed = [0, 0]
        speed_msg = self.ros_message_from_speed(speed)
        self.motor_publisher.publish(speed_msg)

        #debug wait
        time.sleep(0.2)

        

    #Should have a listener for QR. If QR


if __name__ == '__main__':
    rospy.init_node('move_control', anonymous=True)
    rate = rospy.Rate(20)
    c = MoveControl()
    c.start_listeners()

