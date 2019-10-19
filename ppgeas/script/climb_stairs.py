#!/usr/bin/env python
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from ppgeas.msg import ArmsSetPoint
import time

class RosiNodeClass():

	# class attributes
	max_arms_rotational_speed = 0.52 # in [rad/s]
	max_translational_speed = 3 # in [m/s]

	# def free_spin(self, front_dir, rear_dir):
	# 	if front_dir == "ccw":
	# 		front_speed = -0.7*self.max_arms_rotational_speed
	# 	elif front_dir == "cw":
	# 		front_speed = 0.7*self.max_arms_rotational_speed
	# 	else:
	# 		front_speed = 0

	# 	if rear_dir == "ccw":
	# 		rear_speed = -0.7*self.max_arms_rotational_speed
	# 	elif rear_dir == "cw":
	# 		rear_speed = 0.7*self.max_arms_rotational_speed
	# 	else:
	# 		rear_speed = 0

	# 	front_right_rotSpeed = front_speed
	# 	front_left_rotSpeed = front_speed
	# 	rear_right_rotSpeed = rear_speed
	# 	rear_left_rotSpeed = rear_speed
			
	# 	arm_command_list = RosiMovementArray()
	# 	# mounting the lists
	# 	for i in range(4):

	# 		# ----- treating the arms commands		
	# 		arm_command = RosiMovement()

	# 		# mounting arm command list
	# 		arm_command.nodeID = i+1
			
	# 		# separates each arm side command
	# 		if i == 0:
	# 			arm_command.joint_var = front_right_rotSpeed
	# 		elif i == 2:
	# 			arm_command.joint_var = front_left_rotSpeed
	# 		elif i == 1:
	# 			arm_command.joint_var = rear_right_rotSpeed
	# 		else:
	# 			arm_command.joint_var = rear_left_rotSpeed

	# 		# appending the command to the list
	# 		arm_command_list.movement_array.append(arm_command)

	# 	# publishing
	# 	self.pub_arm.publish(arm_command_list)

	# class constructor
	def __init__(self):

		self.arm_front_right_rotSpeed = 0
		self.arm_front_left_rotSpeed = 0
		self.arm_rear_right_rotSpeed = 0
		self.arm_rear_left_rotSpeed = 0
		#self.arm_front_setPoint = -2.35
		#self.arm_rear_setPoint = -0.78
		self.arm_front_right_setPoint = -0.2#-2.35
		self.arm_front_left_setPoint = 0.2#2.35
		self.arm_rear_right_setPoint = 0#3#-np.pi/2
		self.arm_rear_left_setPoint = 0#-3#np.pi/2
		self.arm_front_right_position = 0
		self.arm_front_left_position = 0
		self.arm_rear_right_position = 0
		self.arm_rear_left_position = 0
		self.omega_right = 0
		self.omega_left = 0
		self.sentido_giro_frente = "ccw"
		self.sentido_giro_atras = "ccw"
		self.arms_speed = 0
		self.state = 0
		self.count_correct_angle = 0
		# sends a message to the user
		rospy.loginfo('Control_arm_speed node started')

		# registering to publisher
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		self.pub_Arm_sp = rospy.Publisher('/arm_sp', ArmsSetPoint, queue_size=1)
		
		# registering to subscribers
		self.sub_Arm_position = rospy.Subscriber('/rosi/arms_joints_position', RosiMovementArray, self.callback_Arm_position)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		while not rospy.is_shutdown():
			if self.state == 0:
				rospy.loginfo('Estado 0: Setando as esteiras')
				while True:
					arm_sp_msg = ArmsSetPoint()
					arm_sp_msg.front_dir = 0
					arm_sp_msg.rear_dir = 1
					arm_sp_msg.front_arms_sp = 3.5
					arm_sp_msg.rear_arms_sp = 0.15
					self.pub_Arm_sp.publish(arm_sp_msg)
					if (abs(self.arm_front_right_position) > 3.0) \
					and (abs(self.arm_rear_right_setPoint - self.arm_rear_right_position) < 1e-1):
						break
				self.state += 1

			elif self.state == 1:
				rospy.loginfo('Estado 1: Indo pra frente')
				self.state = 1

			node_sleep_rate.sleep()

	def callback_Arm_position(self, msg):

		# saving_arms_positions (feedback error)
		if (msg.movement_array[0].joint_var < 0):
			self.arm_front_right_position = 2 * np.pi + msg.movement_array[0].joint_var
		else:
			self.arm_front_right_position = msg.movement_array[0].joint_var

		if (msg.movement_array[2].joint_var > 0):
			self.arm_front_left_position = -2*np.pi + msg.movement_array[2].joint_var
  		else:
			self.arm_front_left_position = msg.movement_array[2].joint_var

		if (msg.movement_array[1].joint_var > 0):
			self.arm_rear_right_position = -2*np.pi + msg.movement_array[1].joint_var
		else:
			self.arm_rear_right_position = msg.movement_array[1].joint_var

		if (msg.movement_array[3].joint_var < 0):
			self.arm_rear_left_position =  2*np.pi + msg.movement_array[3].joint_var
		else:
			self.arm_rear_left_position = msg.movement_array[3].joint_var

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('climb_stairs_node', anonymous=True)

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass

