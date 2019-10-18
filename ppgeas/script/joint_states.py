#!/usr/bin/env python
#
## Codigo comentado
#

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rosi_defy.msg import ManipulatorJoints

class RosiNodeClass():

    def callback_EstadoManipulador(self, msg):
        self.states = msg

    def __init__(self):
        # Subscriber estado atual do manipulador
        self.sub = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_EstadoManipulador)
        # Publisher do estado estado atual para JointState
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        # Inicializacao da variavel para receber o valor das juntas do manipulador
        self.states = ManipulatorJoints()
        # Inicializacao da variavel para receber os valores das juntas
        self.juntas = JointState()
        self.juntas.header = Header()
        self.juntas.header.stamp = rospy.Time.now()
        self.juntas.name = ['joint_ur5_j1', 'joint_ur5_j2', 'joint_ur5_j3', 'joint_ur5_j4', 'joint_ur5_j5', 'joint_ur5_j6']
        self.juntas.position = []
        self.juntas.velocity = []
        self.juntas.effort = []
        # Frequencia de execucao do no
        self.rate = rospy.Rate(10)
        # Loop a ser executado para atualizacao da posicao das juntas (entre planejador e simulador)
        while not rospy.is_shutdown():
            self.juntas.header.stamp = rospy.Time.now()
            self.juntas.position = self.states.joint_variable
            self.pub.publish(self.juntas)
            self.rate.sleep()

if __name__ == '__main__':
    # Inicia o no
	rospy.init_node('joint_state_publisher', anonymous=True)

	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass
