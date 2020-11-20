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
    def __init__(self):
        self.pd = PD()
        self.motor_publisher = rospy.Publisher('motor_control', JointState, queue_size=1)
        self.follow_lock = threading.Lock()
        self.angle_lock = threading.Lock()
        self.qr_lock = threading.Lock()

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
        msg.name = ["left wheel", "right_wheel"]
        msg.velocity = speed
        return msg
    # Should have a listener for angle which just updates the current angle on incoming messages, check if running 
    def start_listeners(self):
		
        angle_sub = rospy.Subscriber("mv", Int32, self._angle_callback)
        qr_sub = rospy.Subscriber("mv_qr", JointState, self._qr_callback)
        
        

    def stop(self):
        with self.follow_lock:
            if self.follow:
                self.follow = False
                self._stop_robot()
    
    def _stop_robot(self):
        speed_msg = self.ros_message_from_speed([0,0])
        self.motor_publisher.publish(speed_msg)

    def set_motors_from_angle(self):
        #If angle hasn't changed, then we don't have to update anything
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.angle_lock:
                angle = self.angle
            
            #if angle == self.prev_angle: #This might lock here if the robot speed has been set elsewhere and the angle does not change
            #    continue
            #self.prev_angle = angle

            with self.follow_lock:
                follow = self.follow

            if follow:
                speed = self.pd.speed_from_angle(angle)
                speed_msg = self.ros_message_from_speed(speed)
                self.motor_publisher.publish(speed_msg)
            
            


    def process_qr(self):
        #Only run this procedure once per intersection.
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            #rospy.logwarn("In QR while")
            rate.sleep()
            with self.qr_lock:
                qr = self.qr
            
            if self.qr.name[0] == self.prev_qr.name[0]:
                #rospy.logwarn("Before CONTINUE!!!")
                continue
            self.prev_qr = qr
            rospy.logwarn(self.prev_qr)
            data = qr.name[0]
            x = qr.position[0]
            y = qr.position[1]
            from_dir = int(qr.effort[0])

            #outside_goal_rect = not ((0.2 < x < 0.8) and (0.2 < x < 0.8))

            #Turn off line following while we do the qr processing
            with self.follow_lock:
                self.follow = False

            self._stop_robot()
            print("qr found in goal rect, stopping robot")
            #time.sleep(3)


            #Blocking call waiting for a new move instruction
            if self.qr.name[0] == "{0,1}":
                rospy.logwarn("found the goal yo")
                self._stop_robot()
                time.sleep(200)
            #Here we then want to run a procedure to turn left/right/forward etc. For debug purposes we just try to turn left
            else:
                self.turn_left()

            #Start the line following again and turn off qr_processing mode
            with self.follow_lock:
                self.follow = True




    def turn_left(self):
        self._drive([30, 621], 4)

    def turn_right(self):
        self._drive([621, 30], 4)

    def drive_forward(self):
        self._drive([621, 621], 4)

    #method to turn around is also needed

    #Will set the motors to a certain speed for x sec
    def _drive(self, speed, time_to_drive):
        #Set motor to speed
        speed_msg = self.ros_message_from_speed(speed)
        self.motor_publisher.publish(speed_msg)

        #Let it turn for time_to_drive seconds
        time.sleep(time_to_drive)


        #Then stop again
        self._stop_robot()

        #debug wait
        time.sleep(3)

        

    #Should have a listener for QR. If QR


if __name__ == '__main__':
	
	rospy.init_node('move_control', anonymous=True)
	
	c = MoveControl()
	c.start_listeners()
	
	angle_thread = threading.Thread(target=c.set_motors_from_angle)
	rospy.logwarn("After ang_thread")
	qr_thread = threading.Thread(target=c.process_qr)
	rospy.logwarn("After qr_thread")
	
	angle_thread.start()
	rospy.logwarn("After ang_start")
	qr_thread.start()
	rospy.logwarn("After qr_start")
	print("threads started")
	angle_thread.join()
	qr_thread.join()
	rospy.spin()


