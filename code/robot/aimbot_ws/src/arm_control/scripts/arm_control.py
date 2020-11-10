
import rospy
import time
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

import functions






pub = rospy.Publisher('motor_data', JointState, queue_size=10)



def arm_control(data):

	print("Recieved data: " + str(data.data))

	
	file_name = "give_to_ed.txt"


	path = functions.read_path(file_name)
	
	joint = JointState()
	for i in path:
		angles = i
				
		joint.header.stamp = rospy.Time.now()
		joint.position = angles
		joint.velocity = [10, 10, 10, 10, 10]
		joint.name = ["1", "2", "3", "4", "5"]
		
		pub.publish(joint)
		
		time.sleep(5)

	print("Done with: " + str(data.data))


def listener(): 	# Set up subscriber to ros-node
    # Initialization
    rospy.init_node('arm_control')
    base_sub = rospy.Subscriber("AH", Int32, arm_control)
    
    
    #ats = ApproximateTimeSynchronizer([base_sub], queue_size=2, slop=0.1)
    #ats.registerCallback(send_to_motors)
    rospy.spin()
#####################################    
 

    
######################################    
if __name__ == '__main__':
    listener()
