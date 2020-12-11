#!/usr/bin/env python3

import threading
import rospy
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
import time

LEFT_WHEEL_MOTOR = "6"
RIGHT_WHEEL_MOTOR = "7"

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
        speed_R=min(max(self.speed+cont,0),1023)
        speed_L=min(max(self.speed-cont,0),1023)		
        return [speed_L, speed_R]


			
class MoveControl:
    angle = 90
    prev_angle = 90
    follow = True
    qr = JointState()
    qr.name = ["start"]
    qr.position = [0,0]
    qr.effort = [0]
    prev_qr = qr
    line_follow_thread = None
    qr_thread = None
    
    def __init__(self):
        self.pd = PD()
        self.motor_publisher = rospy.Publisher('motor_control', JointState, queue_size=1)
        self.follow_lock = threading.Lock()
        self.angle_lock = threading.Lock()
        self.qr_lock = threading.Lock()
        self.qr_thread = threading.Thread(target=self._stop_on_new_qr)
        self.qr_thread.start()

    def _angle_callback(self, angle):
        with self.angle_lock:
            self.angle = angle.data

    def _qr_callback(self, qr):
        with self.qr_lock:
            self.qr = qr
    
    #Speed should be a list of two ints. [left speed, right speed]
    def ros_message_from_speed(self, speed):
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.name = [LEFT_WHEEL_MOTOR, RIGHT_WHEEL_MOTOR]
       

        #TODO The order of the wheel speeds seems to be wrong somewhere, so i switched them here
        #msg.velocity = speed
        msg.velocity = [speed[1], speed[0]]
        #rospy.logwarn(speed)
        return msg

    # Should have a listener for angle which just updates the current angle on incoming messages, check if running 
    def init_listeners(self):	
        angle_sub = rospy.Subscriber("mv", Int32, self._angle_callback)
        qr_sub = rospy.Subscriber("mv_qr", JointState, self._qr_callback)
    
    # TODO: start_listeners and init_services should not be class methods.
    def init_services(self):
        rospy.Service('follow_line', Empty, self.follow_line)
        rospy.Service('stop', Empty, self.stop_service)
        rospy.Service('drive_forward', Empty, self.drive_forward)
        rospy.Service('turn_left', Empty, self.turn_left)
        rospy.Service('turn_right', Empty, self.turn_right)
        rospy.Service('turn_back', Empty, self.turn_back)
    
        
    # Public service. Will start a new line follow thread which updates the speed until cancelled
    def follow_line(self, req):
        with self.follow_lock:
            self.follow = True
        # Start background line follower in a separate thread
        if self.line_follow_thread == None or not self.line_follow_thread.is_alive():
            self.line_follow_thread = threading.Thread(target=c._background_line_follower)
            self.line_follow_thread.start()
        return ()
    
    def stop_service(self, req):
        self.stop()
        return ()
    # This should be executable as a service from other nodes. Will stop the robot and line following        
    def stop(self):
        with self.follow_lock:
            if self.follow:
                self.follow = False
                self._stop_robot()
    
    def _stop_robot(self):
        speed_msg = self.ros_message_from_speed([0,0])
        self.motor_publisher.publish(speed_msg)
            
        
        

    # This function will run a continous line following procedure, updates motor speeds in correct frequency for PD (20hz).
    # Cancelable by setting self.follow = False
    def _background_line_follower(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.follow_lock:
                if not self.follow:
                    #print("Line following stopped")
                    return

            speed = self.pd.speed_from_angle(self.angle)
            speed_msg = self.ros_message_from_speed(speed)
            self.motor_publisher.publish(speed_msg)
            
            rate.sleep()
            
            
    
    # This runs in a background thread and stops the robot (and line following) if a new qr is detected. 
    def _stop_on_new_qr(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.qr_lock:
                qr = self.qr
            
            if self.qr.name[0] == self.prev_qr.name[0]:
                continue
            self.prev_qr = qr

            with self.follow_lock:
                self.follow = False    
                self._stop_robot()


    # Public service
    def turn_right(self, req):
        self._drive([30, 621], 4)
        return ()

    # Public service
    def turn_left(self, req):
        self._drive([621, 30], 4)
        return ()

    # Public service
    def drive_forward(self, req):
        self._drive([621, 621], 2)
        return ()

    # Public service
    # These values are totally untested, but should work something like this
    def turn_back(self, req):
        self._drive([-400, -400], 4)
        self._drive([500, -500], 5.4)
        return ()
    
    #Will set the motors to a certain speed for x sec
    def _drive(self, speed, time_to_drive):
        
        #Make sure robot is stopped before starting
        self.stop()
        #Set motor to speed
        speed_msg = self.ros_message_from_speed(speed)
        self.motor_publisher.publish(speed_msg)

        #Let it turn for time_to_drive seconds
        time.sleep(time_to_drive)


        #Then stop again
        self._stop_robot()

        #debug wait
        time.sleep(1)


if __name__ == '__main__':
	
	rospy.init_node('move_control', anonymous=True)
	
	c = MoveControl()
	c.init_listeners()
	c.init_services()
	print("reached inits ok")

	rospy.spin()


