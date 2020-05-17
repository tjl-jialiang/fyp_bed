#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import actionlib
import learnjoy_python.msg  

# Intializes everything
def start():
	#Publisher for bed velocity
	global pub
	pub = rospy.Publisher('bed_cmd_vel',Twist,queue_size = 1)
	
	#Publisher for autonomous mode toggle
	global pub_enAuto
	pub_enAuto = rospy.Publisher('AutoMode',Bool,queue_size = 1,latch = True)
	
	rospy.init_node('teleop')	
	pub_enAuto.publish(toggleAuto) #Set default to not auto first
	
	rospy.Subscriber("joy", Joy, callback) #subscribe to joystick node
	rospy.Subscriber("imuyaw", Float32, yawcb) #subscribe to imu yaw angle
	
	#action client for turning
	global ac
	ac = actionlib.SimpleActionClient('turnserver', learnjoy_python.msg.turnAction)
	ac.wait_for_server()
	
	rospy.spin()


#################


def callback(data):
	global toggleAuto
	global goalYaw
	global offset	
	global turningNow #to ensure one goal processed each time only
	
	if data.buttons[0] == 1: #'A' button
		if turningNow == 1:
			ac.cancel_goal()
		toggleAuto = not toggleAuto
		pub_enAuto.publish(toggleAuto)
		#rospy.loginfo('A button turningFlag is %d' % turningNow) #Tested: did not change flag here
		return None
		
	if data.buttons[1] == 1 and toggleAuto == 1 and turningNow == 0: #button B
		if data.axes[0] == 0 and data.axes[1] == 0: #no joystick input to indicate dir of rotation
			return None
		#derive angle wrt vertical axis of joystick i.e push up is zero degree, push left is rotate CCW by 90 degrees
		anglein = math.atan2(data.axes[1],data.axes[0])
		if anglein < -(math.pi / 2):
			goalYaw = -3*math.pi/2 - anglein
		else:	
			goalYaw = (math.pi / 2) - math.atan2(data.axes[1],data.axes[0]) 
		
		#rospy.loginfo('Start angle:%.9f x:%f y:%f goalYaw:%f' % (yaw, data.axes[0], data.axes[1], goalYaw) )
		
		offset = yaw
		
		if goalYaw > 0:
			rotDir = 1
		else:
			rotDir = -1
		
		goal = learnjoy_python.msg.turnGoal()
		goal.rotDir = rotDir		
		
		#sendgoal
		ac.send_goal(goal, done_cb=turnDone_cb, active_cb=turnActive_cb, feedback_cb=turnFeedb_cb )
		pub_enAuto.publish(0) #Disable auto corridor centering mode
		turningNow = 1
		rospy.loginfo('Start turn by %f' % (math.degrees(goalYaw)) )
		return None	
	
	if toggleAuto == 0:
		twist = Twist()
		twist.linear.x = -1.5*data.axes[0]
		twist.linear.y = 1.5*data.axes[1]
		twist.angular.z = 9*data.axes[3]
		pub.publish(twist)


#################


def yawcb(data):
	global yaw
#	global turningNow
#	global firstYaw
	#Convert from (-pi/2 - pi/2) to (0 - 2*pi)
	if data.data < 0:
		yaw = round(data.data + (2*math.pi),2) #Need to round cuz turnAction feedback_cb compares angle, sensor fusion result jumps around!
	else: 
		yaw = round(data.data,2)
	
	
"""
	if data.data < 0:
		angle = data.data + (2*math.pi) 
	else: 
		angle = data.data
		
	if firstYaw == 1:
		yaw = angle
		return None

	if turningNow == 1: #Purpose of this segment: When starting to turn eg. CCW, sensor might return current angle that is less than offset angle, terminating turnAction prematurely when feedback_cb is comparing angles.
		if goalYaw > 1: #CCW direction
			if yaw - angle > 300: #wrap around 360 problem
				yaw = angle
				return None 
			if angle > yaw:
				yaw = angle
				return None
		else:		#CW direction
			if angle - yaw > 300:
				yaw = angle
				return None			
			if angle < yaw:
				yaw = angle
				return None	
"""	
	
	

#################

#Callbacks for sending goal in turn action  
def turnDone_cb(state,result):
	global turningNow
	global yaw
	#rospy.loginfo('Stopped turning with state %d' %state)
	turningNow = 0
	twist = Twist()
	pub.publish(twist)
	rospy.loginfo('End angle:%.9f' % yaw )
	pub_enAuto.publish(1) #Re-enable auto corridor centering mode
	return
	
def turnActive_cb():
	global goalYaw 
	#rospy.loginfo('Begin turn by %f' % goalYaw) #Tested: did not reach active

def turnFeedb_cb(feedback):
	#rospy.loginfo('At feedback callback')
	global goalYaw #how much to turn by
	global offset #yaw angle at time of initiating turn
	global yaw #constantly update yaw from IMU
	global pub
	#print('offset at actionfeedback: %f' % offset)
	current = yaw - offset #current yaw relative to yaw at start of turn
	#print(current)
	if feedback.contRot > 0: #2 different cases to account for for the 2 direction of rotation
		if current < 0:
			current = current + (2*math.pi)	#to resolve wrap around 0-360 issue
		if current < goalYaw:
			twist = Twist()
			twist.angular.z = 9*feedback.contRot
			pub.publish(twist)
			#rospy.loginfo('sending CCW rotation')
		else: #reached goal, pre-empt
			ac.cancel_goal()
			#rospy.loginfo('cancelled CCW rotation')
	else:
		if current > 0:
			current = current - (2*math.pi)	#to resolve wrap around 0-360 issue
		if current > goalYaw:
			twist = Twist()
			twist.angular.z = 9*feedback.contRot
			pub.publish(twist)
			#rospy.loginfo('sending CW rotation')
		else: #reached goal, pre-empt
			ac.cancel_goal()
			#rospy.loginfo('cancelled CW rotation')
		

#################

if __name__ == '__main__':
	toggleAuto = 0
	turningNow = 0
	firstYaw = 1
	start()
	
