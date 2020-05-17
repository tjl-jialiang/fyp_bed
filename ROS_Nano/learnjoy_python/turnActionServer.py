#!/usr/bin/env python

import rospy
import actionlib
import learnjoy_python.msg
from std_msgs.msg import Bool

class turning(object):
	#C8 msgs to publish result and feedback
	_feedback = learnjoy_python.msg.turnFeedback()
	_result = learnjoy_python.msg.turnResult()
	
	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, learnjoy_python.msg.turnAction, execute_cb = self.execute_cb, auto_start = False)
		self._as.start()
		#self._pub_enAuto = rospy.Publisher('AutoMode',Bool,queue_size = 1,latch = True)
		
	def execute_cb(self,goal):
		r = rospy.Rate(10) 
		#self._pub_enAuto.publish(0) #Disable auto corridor centering mode
		#success = True
		rospy.loginfo('Executing turn')
		self._feedback.contRot = goal.rotDir
		
		while True:
			if self._as.is_preempt_requested():  #requested by client when angle reached
				self._result.stopRot = 1
				rospy.loginfo('Preempting')
				self._as.set_preempted(self._result)
				#success = False
				break
			
			self._as.publish_feedback(self._feedback)
			r.sleep()
		
if __name__ == '__main__':
    rospy.init_node('turnserver')
    server = turning(rospy.get_name())
    rospy.spin()
