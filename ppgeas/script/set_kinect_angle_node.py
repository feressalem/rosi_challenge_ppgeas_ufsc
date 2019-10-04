#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32

class SetKinectAngle():

	# class constructor
	def __init__(self):
		# sends a message to the user
		rospy.loginfo('Set_kinect_angle node started')

		# registering to publishers
		self.pub_angle = rospy.Publisher('rosi/command_kinect_joint', Float32, queue_size=1)

		# registering to subscribers
		self.sub_angle = rospy.Subscriber('/rosi/kinect_joint', Float32, self.callback_Angle)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		while not rospy.is_shutdown():

			#angle_command = Float32()
			#angle_command = -0.174533

			#self.pub_angle.publish(angle_command)		
			
			# sleeps for a while
			node_sleep_rate.sleep()
	
	# angle callback function
	def callback_Angle(self, msg):
		angle = msg.data
		angle_command = Float32()
		delta_angle = - 0.174533 - angle
		if abs(delta_angle) >= 0.01 :
			angle_command = -0.174533
			self.pub_angle.publish(angle_command)

		
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('set_kinect_angle_node', anonymous=True)

	# instantiate the class
	try:
		node_obj = SetKinectAngle()
	except rospy.ROSInterruptException: pass

