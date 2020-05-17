#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


# Intializes everything
def start():
	global pub
	pub = rospy.Publisher('bed_cmd_vel',Twist,queue_size = 1)
	global pub_enAuto
	pub_enAuto = rospy.Publisher('AutoMode',Bool,queue_size = 1,latch = True)
	
	rospy.init_node('Joy2Arduino')	
	pub_enAuto.publish(toggleAuto)
	rospy.Subscriber("joy", Joy, callback)
	
	rospy.spin()

def callback(data):
	global toggleAuto
	if data.buttons[0] == 1:	#TODO check button mapping
		toggleAuto = not toggleAuto
		pub_enAuto.publish(toggleAuto)
	if toggleAuto == 0:
		twist = Twist()
		twist.linear.x = -1.5*data.axes[0]
		twist.linear.y = 1.5*data.axes[1]
		twist.angular.z = 9*data.axes[3]
		pub.publish(twist)


if __name__ == '__main__':
	toggleAuto = 0
	start()
	
