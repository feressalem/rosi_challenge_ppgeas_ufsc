#!/usr/bin/env python
#
## Codigo comentado
#

import rospy
import numpy as np
import math
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from ppgeas.msg import ArmsSetPoint


class RosiNodeClass():

	# class attributes
	max_arms_rotational_speed = 0.52 # in [rad/s]


	# class constructor
	def __init__(self):

		self.arm_front_right_rotSpeed = 0
		self.arm_front_left_rotSpeed = 0
		self.arm_rear_right_rotSpeed = 0
		self.arm_rear_left_rotSpeed = 0
		self.arm_direction = 0
		self.arm_front_setPoint = 0
		self.arm_rear_setPoint = 0
		self.arm_front_right_position = 0
		self.arm_front_left_position = 0
		self.arm_rear_right_position = 0
		self.arm_rear_left_position = 0

		# sends a message to the user
		rospy.loginfo('Control_arm_speed node started')

		# registering to publisher
		self.pub_arm = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)

		# registering to subscribers
		self.sub_Arm_sp = rospy.Subscriber('/arm_sp', ArmsSetPoint, self.callback_Arm_sp)
		self.sub_Arm_position = rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, self.callback_Arm_position)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		while not rospy.is_shutdown():
			if self.arm_direction == 0:
				if (abs(self.arm_front_setPoint-self.arm_front_right_position)<0.06):
					self.arm_front_right_rotSpeed = 0
				elif(self.arm_front_setPoint < self.arm_front_right_position):
					self.arm_front_right_rotSpeed = -((2*math.pi + self.arm_front_setPoint) - self.arm_front_right_position)
				else:
					self.arm_front_right_rotSpeed = -(self.arm_front_setPoint - self.arm_front_right_position)

				if (abs(self.arm_front_setPoint-abs(self.arm_front_left_position))<0.06):
					self.arm_front_left_rotSpeed = 0
				elif(self.arm_front_setPoint < abs(self.arm_front_left_position)):
					self.arm_front_left_rotSpeed = -((2*math.pi + self.arm_front_setPoint) - abs(self.arm_front_left_position))
				else:
					self.arm_front_left_rotSpeed = -(self.arm_front_setPoint - abs(self.arm_front_left_position))
				
				if (abs(self.arm_rear_setPoint-abs(self.arm_rear_right_position))<0.06):
					self.arm_rear_right_rotSpeed = 0
				elif(self.arm_rear_setPoint < abs(self.arm_rear_right_position)):
					self.arm_rear_right_rotSpeed = ((2*math.pi + self.arm_rear_setPoint) - abs(self.arm_rear_right_position))
				else:
					self.arm_rear_right_rotSpeed = (self.arm_rear_setPoint - abs(self.arm_rear_right_position))
				
				if (abs(self.arm_rear_setPoint-self.arm_rear_left_position)<0.06):
					self.arm_rear_left_rotSpeed = 0
				elif(self.arm_rear_setPoint < self.arm_rear_left_position):
					self.arm_rear_left_rotSpeed = ((2*math.pi + self.arm_rear_setPoint) - self.arm_rear_left_position)
				else:
					self.arm_rear_left_rotSpeed = (self.arm_rear_setPoint - self.arm_rear_left_position)
			
			else:
				if (abs(self.arm_front_setPoint-self.arm_front_right_position)<0.06):
					self.arm_front_right_rotSpeed = 0
				elif(self.arm_front_setPoint  < self.arm_front_right_position):
					self.arm_front_right_rotSpeed = (self.arm_front_right_position - self.arm_front_setPoint)
				else:
					self.arm_front_right_rotSpeed = ((2*math.pi + self.arm_front_right_position) - self.arm_front_setPoint)
				
				if (abs(self.arm_front_setPoint-abs(self.arm_front_left_position))<0.06):
					self.arm_front_left_rotSpeed = 0
				elif(self.arm_front_setPoint  < abs(self.arm_front_left_position)):
					self.arm_front_left_rotSpeed = (abs(self.arm_front_left_position) - self.arm_front_setPoint)
				else:
					self.arm_front_left_rotSpeed = ((2*math.pi + abs(self.arm_front_left_position)) - self.arm_front_setPoint)			

				if (abs(self.arm_rear_setPoint-abs(self.arm_rear_right_position))<0.06):
					self.arm_rear_right_rotSpeed = 0
				elif(self.arm_rear_setPoint  < abs(self.arm_rear_right_position)):
					self.arm_rear_right_rotSpeed = -(abs(self.arm_rear_right_position) - self.arm_rear_setPoint)
				else:
					self.arm_rear_right_rotSpeed = -((2*math.pi + abs(self.arm_rear_right_position)) - self.arm_rear_setPoint)	
				
				if (abs(self.arm_rear_setPoint-self.arm_rear_left_position)<0.06):
					self.arm_rear_left_rotSpeed = 0
				elif(self.arm_rear_setPoint  < self.arm_rear_left_position):
					self.arm_rear_left_rotSpeed = -(self.arm_rear_left_position - self.arm_rear_setPoint)
				else:
					self.arm_rear_left_rotSpeed = -((2*math.pi + self.arm_rear_left_position) - self.arm_rear_setPoint)
			

			#self.arm_rear_left_rotSpeed = 0			
			print(self.arm_rear_left_rotSpeed)
			arm_command_list = RosiMovementArray()
			# mounting the lists
			for i in range(4):

				# ----- treating the arms commands
				arm_command = RosiMovement()

				# mounting arm command list
				arm_command.nodeID = i+1

				# separates each arm side command
				if i == 0:
					arm_command.joint_var = 2 * self.arm_front_right_rotSpeed
				elif i == 2:
					arm_command.joint_var = 2 * self.arm_front_left_rotSpeed
				elif i == 1:
					arm_command.joint_var = 2 * self.arm_rear_right_rotSpeed
				else:
					arm_command.joint_var = 2 * self.arm_rear_left_rotSpeed

				# appending the command to the list
				arm_command_list.movement_array.append(arm_command)

			# publishing
			self.pub_arm.publish(arm_command_list)

			# sleeps for a while
			node_sleep_rate.sleep()

		# infinite loop
		#while not rospy.is_shutdown():
			# pass

		# enter in rospy spin
		#rospy.spin()

	# joystick callback function
	def callback_Arm_sp(self, msg):
		self.arm_direction = msg.dir
		# saving_arms_setpoints
		self.arm_front_setPoint = msg.front_arms_sp
		self.arm_rear_setPoint = msg.rear_arms_sp

	def callback_Arm_position(self, msg):

		# saving_arms_positions (feedback error)
		if (msg.movement_array[0].joint_var < 0):
			self.arm_front_right_position = 2 * math.pi + msg.movement_array[0].joint_var
		else:
			self.arm_front_right_position = msg.movement_array[0].joint_var

		if (msg.movement_array[2].joint_var > 0):
			self.arm_front_left_position = -2*math.pi + msg.movement_array[2].joint_var
  		else:
			self.arm_front_left_position = msg.movement_array[2].joint_var

		if (msg.movement_array[1].joint_var > 0):
			self.arm_rear_right_position = -2*math.pi + msg.movement_array[1].joint_var
		else:
			self.arm_rear_right_position = msg.movement_array[1].joint_var

		if (msg.movement_array[3].joint_var < 0):
			self.arm_rear_left_position =  2*math.pi + msg.movement_array[3].joint_var
		else:
			self.arm_rear_left_position = msg.movement_array[3].joint_var

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('control_arm_speed', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass
