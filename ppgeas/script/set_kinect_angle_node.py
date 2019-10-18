#!/usr/bin/env python
#
## Codigo comentado
#

import rospy
import numpy as np
from std_msgs.msg import Float32

class SetKinectAngle():

	# Construturo de classe
	def __init__(self):
		# Envia uma mensagem ao usuario
		rospy.loginfo('Set_kinect_angle node started')

		# Registra o publisher para a movimentacao do Kinect
		self.pub_angle = rospy.Publisher('rosi/command_kinect_joint', Float32, queue_size=1)

		# Registra o Subscriber para o angulo do Kinect
		self.sub_angle = rospy.Subscriber('/rosi/kinect_joint', Float32, self.callback_Angle)

		# Define a frequencia de execucao do no
		node_sleep_rate = rospy.Rate(10)

		# Loop
		while not rospy.is_shutdown():
			#angle_command = Float32()
			#angle_command = -0.174533
			#self.pub_angle.publish(angle_command)
			# sleeps for a while
			node_sleep_rate.sleep()

	# metodo de callback para o angulo do kinect
	def callback_Angle(self, msg):
		# Get o angulo
		angle = msg.data
		# Define a variavel de comando
		angle_command = Float32()
		# Calcula a variacao
		delta_angle = - 0.174533 - angle
		if abs(delta_angle) >= 0.01 :
			angle_command = -0.174533
			# Publica o comando para o kinect
			self.pub_angle.publish(angle_command)


if __name__ == '__main__':
	# Inicializa o no
	rospy.init_node('set_kinect_angle_node', anonymous=True)
	# instancia a classe
	try:
		node_obj = SetKinectAngle()
	except rospy.ROSInterruptException: pass
