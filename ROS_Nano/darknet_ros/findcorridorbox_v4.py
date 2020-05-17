#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist

detectedBoxes = BoundingBoxes()
detectedBoxes.header.seq = 1

corridorBox = BoundingBox()

def callback(data):
	global automode
	if (automode == False):
		pub_pidEn.publish(0)
		return None
	
	global detectedBoxes
	detectedBoxes = data
	#rospy.loginfo(detectedBoxes.header.seq)


def timercb(event):
	global seq_prev
	global detectedBoxes
	global automode
	global state_prev
	global state_pp

	if automode == 0:
		state_prev = 0 #reset
		state_pp = 0 #reset
		return None
	if detectedBoxes.header.seq == seq_prev:
		#if no new objects detected at all
		pub_pidEn.publish(0)
		rospy.loginfo('Timercb: No detection since prev')
		#Nd publish zero vel?
		twist = Twist()
		pub.publish(twist)
		return None
	seq_prev = detectedBoxes.header.seq
	corridorBox = BoundingBox()
	firstCorridor = 1
	for box in detectedBoxes.bounding_boxes:
		if box.Class == 'corridor':
			if firstCorridor == 1:
				corridorBox = box
				firstCorridor += 1
			if ((box.xmax-box.xmin) < (corridorBox.xmax-corridorBox.xmin)): #Finding smallest box
				corridorBox = box

	if corridorBox.Class == "":
		pub_pidEn.publish(0)
		twist = Twist()
		pub.publish(twist)
		rospy.loginfo('Timercb: Detected but no corridor')
	else:
		pub_pidEn.publish(1)
		corridorCent = (corridorBox.xmin + corridorBox.xmax) / 2
		pub_state.publish(0.6*corridorCent + 0.3*state_prev + 0.1*state_pp) #smoothing
		state_pp = state_prev
		state_prev = corridorCent
		#rospy.loginfo('Timercb: sent state of %f'% corridorCent)
	

def automode_callback(data):
	global automode	
	automode = data.data
	rospy.loginfo("Auto mode is %s",automode)
	pub_pidEn.publish(automode)
	

def corridorState():
	global pub_state
	pub_state = rospy.Publisher('state', Float64 ,queue_size = 1)
	global pub_pidEn
	pub_pidEn = rospy.Publisher('/pid_enable', Bool ,queue_size = 1)
	global pub
	pub = rospy.Publisher('bed_cmd_vel',Twist,queue_size = 1)
	rospy.init_node('corridorState', anonymous=True)
	rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
	rospy.Subscriber("AutoMode", Bool, automode_callback)
	rospy.Timer(rospy.Duration(0.5),timercb)
	rospy.spin()

if __name__ == '__main__':
	#print imgCentre
	automode = False
	seq_prev = 0
	state_prev = 0
	state_pp = 0
	corridorState()